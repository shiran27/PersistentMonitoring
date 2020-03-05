function Agent(x, y, r) {

    this.position = new Point2(x,y);
    this.orientation = 0;

    this.id = agents.length;

    this.graphicSizeParameter = 20;
    this.graphicColor = color(255,50,50);
    var D = this.graphicSizeParameter;
    this.graphicBaseShape = [new Point2(D,0), new Point2(-D/2,D/2), new Point2(-D/2, -D/2)];// when orientation is zero and located at origin
    this.graphicBaseShapeRotated = this.graphicBaseShape;  // this will chage during the simulations
    this.maxLinearVelocity = 50; 
    this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),this.orientation);

    this.sensingRate = r;
    this.threshold = []; // will be loaded later
    this.initialResidingTarget; // initial placement target id
    this.residingTarget = []; // current taarget or the path[T_i,T_j]
    this.residingTargetUncertainty; // to extract info from the residing target
    this.departureMode = 0; // for IPA
    this.savedEventTimeSensitivity;
    this.sensitivityOfThreshold = []; // will be loaded later

    this.previousTarget; // boosting - old
    this.visitedTargetList = []; // for exploration boosting
    this.unvisitedTargetsInfo = []; // for exploration boosting

    this.timeToExitMode = [3,0];//[sensing=1,t^1+u_i] , [sleeping=2,t^2+v_i], [traveling=3,t^3+rho_ij],  

    this.show = function(){

        fill(this.graphicColor);
        noStroke();
        ////ellipse(this.position.x,this.position.y,0.5*this.graphicSizeParameter,1.5*this.graphicSizeParameter);
        //rect(this.position.x,this.position.y,0.5*this.graphicSizeParameter,1.5*this.graphicSizeParameter);
        var x = this.position.x;
        var y = this.position.y;
        var b = this.graphicBaseShapeRotated;
        ////triangle(x-r/2,y-r/2,x-r/2,y+r/2,x+r,y);
        triangle(b[0].x + x,b[0].y + y, b[1].x + x,b[1].y + y,b[2].x + x,b[2].y + y);

        fill(255);
        rectMode(CENTER);
        textAlign(CENTER,CENTER);
        text((this.id+1).toString(),this.position.x,this.position.y,this.graphicSizeParameter,this.graphicSizeParameter);

    }

    this.clicked = function(){

        var mouseP = new Point2(mouseX,mouseY);
        var d = distP2(mouseP, this.position);
        if (d < this.graphicSizeParameter) {
            return true;
        }
        else{
            return false;
        }

    }
    




    this.updateIPA = function(){
        // update the agent position s_a(t) of agent a
        if(this.residingTarget.length==1){//residing in some target - agent in stationary mode
            
            var i = this.residingTarget[0];

            // split boosting
            if(boostingMethod==4 && boostingMode==1){
                var nextTarget = this.findNextTargetSplitBoost(i);
            }else{
                var nextTarget = this.findNextTarget(i);
            }
            // end split boosting


            // event printing
            if(nextTarget==i){// residing target uncertainty reaching 0
                if(this.residingTargetUncertainty>0 && targets[i].uncertainty==0){// Event 3 Triggered when uncertainty was updated
                    if(printMode){print("Ev "+eventCount+" E 3: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");}
                    sensitivityUpdateAtEvent();
                    this.IPAComputationEventE3(i);
                }
            }else if(nextTarget!=i){// residing target uncertainty increasing from 0
                if(this.getImmediateNeighbors().length==0 && targets[i].uncertainty==0){// Event4 will be triggered next
                    if(printMode){print("Ev "+eventCount+" E 4: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");}
                    sensitivityUpdateAtEvent();// event occured
                    this.IPAComputationEventE4(i);
                }
            }
            // end event printing



            var j = nextTarget; 

            if(targets[i].uncertainty > this.threshold[i][i] || j==i){ //stay
                // stay at i to further reduce the uncertainty
                this.position = this.position;
                this.residingTargetUncertainty = targets[this.residingTarget[0]].uncertainty;
            
                // exploration boosting
                
                if(boostingMethod==5 && boostingMode == 1){
                    if(targets[i].uncertainty < this.threshold[i][i] && targets[i].uncertainty){
                        var maxR = 0;
                        var bestNeighbor = -1;
                        for(var k = 0; k<targets[i].neighbors.length; k++){
                            var neighborTarget = targets[i].neighbors[k];
                            if(neighborTarget!=i && targets[neighborTarget].uncertainty>maxR){
                                maxR = targets[neighborTarget].uncertainty;
                                bestNeighbor = neighborTarget;
                            }
                        }
                        if(bestNeighbor!=-1){
                            this.threshold[i][bestNeighbor] = projectToPositiveAxis(this.threshold[i][bestNeighbor] - 1000*boostingCoefficientAlpha);
                            //this.threshold[bestNeighbor][i] = 0;
                        }
                        
                    }
                }
                
                // end exploration boosting

            }else{ // leave
                // need to start moving in the direction of target j
                // rotate

                // event printing (GIven: targets[i].uncertainty < this.threshold[i][i] && j!=i )
                if(this.residingTargetUncertainty > this.threshold[i][i]){
                    if(printMode){print("Ev "+eventCount+" D 1: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");    }
                    sensitivityUpdateAtEvent();
                    this.departureMode = 1; // use to identify arrival 1 event
                    this.IPAComputationEventD1(i);
                }else if(this.residingTargetUncertainty>0){
                    if(printMode){print("Ev "+eventCount+" D 2: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");  }
                    sensitivityUpdateAtEvent();
                    this.departureMode = 2; // use to identify arrival 2 event
                    this.IPAComputationEventD2(i,j);
                }else{// if(this.residingTargetUncertainty==0){
                    
                    if(targets[i].uncertaintyRate > this.getImmediateNeighborsSensingRate()){
                        if(printMode){print("Ev "+eventCount+" D 3_1: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");      }
                        sensitivityUpdateAtEvent();
                        this.departureMode = 3.1; // use to identify arrival 2 event
                        this.IPAComputationEventD3_1(i,j);
                    }else{
                        if(printMode){print("Ev "+eventCount+" D 3_2: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");      }
                        sensitivityUpdateAtEvent();
                        this.departureMode = 3.2; // use to identify arrival 2 event
                        this.IPAComputationEventD3_2(i,j);
                    }

                }
                this.savedEventTimeSensitivity = eventTimeSensitivity;
                // end event printing


                this.residingTarget = [i,j];
                targets[i].residingAgents[this.id] = false;
                var headingAngle = atan2P2(targets[i].position,targets[j].position);
                var rotationRequired = headingAngle-this.orientation;
                for(var k = 0; k<this.graphicBaseShape.length ; k++){
                    this.graphicBaseShapeRotated[k] = rotateP2(this.graphicBaseShapeRotated[k], rotationRequired);
                }
                this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                this.position = plusP2(this.position, this.headingDirectionStep);
                this.orientation = headingAngle;

                this.residingTargetUncertainty = -1;

                // exploration boosting
                if(boostingMethod==5 && boostingMode == 1){
                    var targetInfo = [];
                    for(var k = 0; k<targets[i].neighbors.length; k++){
                        var neighborTarget = targets[i].neighbors[k]; 
                        if(neighborTarget!=i && neighborTarget !=j){// missed options
                            targetInfo.push([i, neighborTarget, targets[neighborTarget].uncertainty]);
                        }
                    }
                    this.unvisitedTargetsInfo.push(targetInfo); // as we arrived at target j
                }
                // end exploration boosting
            }

        }else{// going from T_i to T_j (as this.residingTarget = [T_i, T_j]) 

            ////print("travelling i to j");
            this.position = plusP2(this.position, this.headingDirectionStep);
            var i = this.residingTarget[0];
            var j = this.residingTarget[1];
            if(distP2(this.position,targets[i].position)>distP2(targets[j].position,targets[i].position)){
                
                // event printing
                if(this.departureMode == 1){
                    if(printMode){print("Ev "+eventCount+" A 1: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(this.residingTarget[1]+1)+";");      }
                    sensitivityUpdateAtEvent();
                    this.IPAComputationEventA1(i,j);
                }else if(this.departureMode == 2){
                    if(printMode){print("Ev "+eventCount+" A 2: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(this.residingTarget[1]+1)+";");      }
                    sensitivityUpdateAtEvent();
                    this.IPAComputationEventA2(i,j);
                }else if(this.departureMode == 3.1){
                    if(printMode){print("Ev "+eventCount+" A 3_1: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(this.residingTarget[1]+1)+";");      }
                    sensitivityUpdateAtEvent();
                    this.IPAComputationEventA2(i,j);
                }else if(this.departureMode == 3.2){
                    if(printMode){print("Ev "+eventCount+" A 3_2: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(this.residingTarget[1]+1)+";");      }
                    sensitivityUpdateAtEvent();
                    this.IPAComputationEventA2(i,j);
                }
                this.departureMode = 0;
                // end event printing

                this.position = targets[j].position;
                this.residingTarget = [j];


                targets[j].residingAgents[this.id] = true;
                this.residingTargetUncertainty = targets[j].uncertainty; 



                // exploration boosting
                if(boostingMethod==5 && boostingMode == 1){
                    this.visitedTargetList.push(j); // as we arrived at target j
                    this.backSearchForCycles(); // search for cycle in the 'this.visitedTargetList' and once identified ---> do appropriate threshold changes and clear the this.targetList
                }
                // end exploration boosting

                // boosting - old approach
                // boosting - test
                // this.previousTarget = i;
                // if(boostingMode==1 && numberOfUpdateStepsCount>300){
                //     var immediateNeighbors = this.getImmediateNeighbors();
                //     if(immediateNeighbors.length>0){
                //         var k = agents[immediateNeighbors[0]].previousTarget;
                //         if(distP2(targets[k].position,targets[j].position)<distP2(targets[i].position,targets[j].position)){
                //             this.threshold[i][j] = this.threshold[i][j]+100;
                //             this.threshold[j][i] = targets[i].uncertainty;
                //             print("booost! Agent: "+(this.id+1)+", Path T_"+(i+1)+" to T_"+(j+1));
                //         }else{
                //             agents[immediateNeighbors[0]].threshold[k][j] = agents[immediateNeighbors[0]].threshold[k][j]+100;
                //             agents[immediateNeighbors[0]].threshold[j][k] = targets[k].uncertainty;;
                //             print("booost! Agent: "+(immediateNeighbors[0]+1)+", Path T_"+(k+1)+" to T_"+(j+1));
                //         }
                //         boostingMode = 0;
                //     }
                // }
                // end boosting

            }
        }
    }



    this.updateCT2 = function(){
        // update the agent position s_a(t) of agent a
        if(this.residingTarget.length==1){//residing in some target - agent in stationary mode
            var i = this.residingTarget[0];
            var nextTarget = this.findNextTarget(i);


            // event printing
            if(nextTarget==i){// residing target uncertainty reaching 0
                if(this.residingTargetUncertainty>0 && targets[i].uncertainty==0){// Event 3 Triggered when uncertainty was updated
                    if(printMode){print("Ev "+eventCount+" E 3: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");}
                    eventCount ++;
                }
            }else if(nextTarget!=i){// residing target uncertainty increasing from 0
                if(this.getImmediateNeighbors().length==0 && targets[i].uncertainty==0){// Event4 will be triggered next
                    if(printMode){print("Ev "+eventCount+" E 4: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");}
                    eventCount ++;
                }
            }
            // end event printing




            var j = nextTarget; 

            if(targets[i].uncertainty > this.threshold[i][i] || j==i){ //stay
                // stay at i to further reduce the uncertainty
                this.position = this.position;
                this.residingTargetUncertainty = targets[this.residingTarget[0]].uncertainty;
            }else{ // leave
                // need to start moving in the direction of target j
                // rotate

                // event printing (GIven: targets[i].uncertainty < this.threshold[i][i] && j!=i )
                if(this.residingTargetUncertainty > this.threshold[i][i]){
                    if(printMode){print("Ev "+eventCount+" D 1: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");    }
                    
                    this.departureMode = 1; // use to identify arrival 1 event
                    
                }else if(this.residingTargetUncertainty>0){
                    if(printMode){print("Ev "+eventCount+" D 2: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");  }
                    
                    this.departureMode = 2; // use to identify arrival 2 event
                    
                }else{// if(this.residingTargetUncertainty==0){
                    
                    if(targets[i].uncertaintyRate > this.getImmediateNeighborsSensingRate()){
                        if(printMode){print("Ev "+eventCount+" D 3_1: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");      }
                        
                        this.departureMode = 3.1; // use to identify arrival 2 event
                        
                    }else{
                        if(printMode){print("Ev "+eventCount+" D 3_2: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(i+1)+";");      }
                        
                        this.departureMode = 3.2; // use to identify arrival 2 event
                        
                    }

                }
                eventCount ++;
                
                // end event printing

                this.residingTarget = [i,j];
                targets[i].residingAgents[this.id] = false;
                var headingAngle = atan2P2(targets[i].position,targets[j].position);
                var rotationRequired = headingAngle-this.orientation;
                for(var k = 0; k<this.graphicBaseShape.length ; k++){
                    this.graphicBaseShapeRotated[k] = rotateP2(this.graphicBaseShapeRotated[k], rotationRequired);
                }
                this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                this.position = plusP2(this.position, this.headingDirectionStep);
                this.orientation = headingAngle;

                this.residingTargetUncertainty = -1;
            }

        }else{// going from T_i to T_j (as this.residingTarget = [T_i, T_j]) 
            ////print("travelling i to j");
            this.position = plusP2(this.position, this.headingDirectionStep);
            var i = this.residingTarget[0];
            var j = this.residingTarget[1];
            if(distP2(this.position,targets[i].position)>distP2(targets[j].position,targets[i].position)){
                
                // event printing
                if(this.departureMode == 1){
                    if(printMode){print("Ev "+eventCount+" A 1: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(this.residingTarget[1]+1)+";");      }
                    
                }else if(this.departureMode == 2){
                    if(printMode){print("Ev "+eventCount+" A 2: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(this.residingTarget[1]+1)+";");      }
                    
                }else if(this.departureMode == 3.1){
                    if(printMode){print("Ev "+eventCount+" A 3_1: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(this.residingTarget[1]+1)+";");      }
                    
                }else if(this.departureMode == 3.2){
                    if(printMode){print("Ev "+eventCount+" A 3_2: t="+simulationTime.toFixed(2)+"; A_a = "+(this.id+1)+"; T_i ="+(this.residingTarget[1]+1)+";");      }
                    
                }


                eventCount ++;
                this.departureMode = 0;
                // end event printing

                this.position = targets[j].position;
                this.residingTarget = [j];

                targets[j].residingAgents[this.id] = true;
                this.residingTargetUncertainty = targets[j].uncertainty; 


                // boosting - test
                // if(boostingMode==1){
                //     var immediateNeighbors = this.getImmediateNeighbors();
                //     if(immediateNeighbors.length>0){
                //         this.threshold[i][j] = this.threshold[i][j]+10;
                //         print("booost! Agent: "+(this.id+1)+", Path T_"+(i+1)+" to T_"+(j+1));
                //     }
                // }

            }
        }
    }





    this.updateFastCT = function(){
        // update the agent position s_a(t) of agent a
        if(this.residingTarget.length==1){//residing in some target
            var i = this.residingTarget[0];
            var j = this.findNextTarget(i);// gives T_j according tho the threshold polcy
            if(targets[i].uncertainty > this.threshold[i][i] || j==i){
                // stay at i to further reduce the uncertainty
                this.position = this.position;
                ////if(dataPlotMode){recordSystemState();}// might be redundent
            }else{
                // need to start moving in the direction of target j
                // rotate
                this.residingTarget = [i,j];
                var headingAngle = atan2P2(targets[i].position,targets[j].position);
                var rotationRequired = headingAngle-this.orientation;
                for(var k = 0; k<this.graphicBaseShape.length ; k++){
                    this.graphicBaseShapeRotated[k] = rotateP2(this.graphicBaseShapeRotated[k], rotationRequired);
                }
                
                // var pid = getPathID(i,j);
                // if(paths[pid].artificiallyExtended){this.maxLinearVelocity = this.maxLinearVelocity/10;}
                
                this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                this.position = plusP2(this.position, this.headingDirectionStep);
                this.orientation = headingAngle;

                if(dataPlotMode){recordSystemState();}// agent Started Moving towards a new target
            }
        }else{// going from T_i to T_j (as this.residingTarget = [T_i, T_j]) 
            ////print("travelling i to j");
            this.position = plusP2(this.position, this.headingDirectionStep);
            if(distP2(this.position,targets[this.residingTarget[0]].position)>distP2(targets[this.residingTarget[1]].position,targets[this.residingTarget[0]].position)){
                this.position = targets[this.residingTarget[1]].position;
                this.residingTarget = [this.residingTarget[1]]; 

                if(dataPlotMode){recordSystemState();}// record agent reached destination target event
                
                // var pid = getPathID(i,j);
                // if(paths[pid].artificiallyExtended){this.maxLinearVelocity = this.maxLinearVelocity*10;}
                
            }else{
                ////if(dataPlotMode){recordSystemState();}// might be redundent
            }
        }
    }


    this.updateCT = function(){
        // update the agent position s_a(t) of agent a
        if(this.residingTarget.length==1){//residing in some target
            var i = this.residingTarget[0];
            var j = this.findNextTarget(i);// gives T_j according tho the threshold polcy
            if(targets[i].uncertainty > this.threshold[i][i] || j==i){
                // stay at i to further reduce the uncertainty
                this.position = this.position;
            }else{
                // need to start moving in the direction of target j
                // rotate
                this.residingTarget = [i,j];
                var headingAngle = atan2P2(targets[i].position,targets[j].position);
                var rotationRequired = headingAngle-this.orientation;
                for(var k = 0; k<this.graphicBaseShape.length ; k++){
                    this.graphicBaseShapeRotated[k] = rotateP2(this.graphicBaseShapeRotated[k], rotationRequired);
                }

                // var pid = getPathID(i,j);
                // if(paths[pid].artificiallyExtended){this.maxLinearVelocity = this.maxLinearVelocity/10;}
                
                ////print("need to go to j; rotated");
                this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                this.position = plusP2(this.position, this.headingDirectionStep);
                this.orientation = headingAngle;
            }
        }else{// going from T_i to T_j (as this.residingTarget = [T_i, T_j]) 
            var i = this.residingTarget[0];// where we were
            var j = this.residingTarget[1];// where we are heading
            var angle = this.orientation;
            ////print("travelling i to j");
            this.position = plusP2(this.position, this.headingDirectionStep);
            if(distP2(this.position,targets[i].position)>distP2(targets[j].position,targets[i].position)){
                ////print("Stopped at j !!! ")
                this.position = targets[j].position;
                this.residingTarget = [j]; 

                // var pid = getPathID(i,j);
                // if(paths[pid].artificiallyExtended){this.maxLinearVelocity = this.maxLinearVelocity*10;}
                
            }
        }
    }



    this.updateRHCCT = function(){
        // update the agent position s_a(t) of agent a
        if(this.residingTarget.length==1){//residing in some target
            var i = this.residingTarget[0];
            // The following ten lines are the only difference compared to "this.updateCT" defined above
            var j;
            if(targets[i].uncertainty>0){
                j = i;
            }else if(RHCMethod==1){
                j = this.findNextTargetOneStepAheadRHC(i);
            }else if(RHCMethod==2){
                j = this.findNextTargetTwoStepsAheadRHC(i);
            }

            if(j==i){
                // stay at i to further reduce the uncertainty till 0
                this.position = this.position;
            }else{
                // need to start moving in the direction of target j
                // rotate
                this.residingTarget = [i,j];
                var headingAngle = atan2P2(targets[i].position,targets[j].position);
                var rotationRequired = headingAngle-this.orientation;
                for(var k = 0; k<this.graphicBaseShape.length ; k++){
                    this.graphicBaseShapeRotated[k] = rotateP2(this.graphicBaseShapeRotated[k], rotationRequired);
                }
                ////print("need to go to j; rotated");
                this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                this.position = plusP2(this.position, this.headingDirectionStep);
                this.orientation = headingAngle;

                if(dataPlotMode){recordSystemState();}// agent Started Moving towards a new target
            }
        }else{// going from T_i to T_j (as this.residingTarget = [T_i, T_j]) 
            var i = this.residingTarget[0];// where we were
            var j = this.residingTarget[1];// where we are heading
            var angle = this.orientation;
            ////print("travelling i to j");
            this.position = plusP2(this.position, this.headingDirectionStep);
            if(distP2(this.position,targets[i].position)>distP2(targets[j].position,targets[i].position)){
                ////print("Stopped at j !!! ")
                this.position = targets[j].position;
                this.residingTarget = [j]; 

                if(dataPlotMode){recordSystemState();}// record agent reached destination target event
            }


        }
    }



    // Evet driven RHC update for an agent
    this.updateEDRHCCT = function(){
        // update the agent position s_a(t) of agent a

        if(this.residingTarget.length==1){//residing in some target
            
            var i = this.residingTarget[0];
            var j = i;

            // only at inital soln
            if(this.timeToExitMode[0]==3 && this.timeToExitMode[1]<simulationTime){
                // solve OP-1 to find the u_i
                var u_i = this.solveOP1(i);
                this.timeToExitMode = [1,simulationTime+u_i];

            }else if(this.timeToExitMode[0]==1){// in sensing mode 
                 if(this.timeToExitMode[1]<simulationTime){//and sensing time elepased
                    if(targets[i].uncertainty>0){// leave early?
                        // solve OP-3 to find the next target j to visit
                        var ans = this.solveOP3(i);
                        j = ans[0];
                        var rho_ij = ans[1]; 
                        if(j!=i){this.timeToExitMode = [3,simulationTime+rho_ij]};
                    }else{
                        // solve OP-2 to find the sleeping time v_i
                        var v_i = this.solveOP2(i);
                        this.timeToExitMode = [2,simulationTime+v_i];
                        //// if v_i = 0, jump directly to moving: Solving OP-3
                    }
                 }else{
                    // waiting till the end of the predetermined sensing period
                    // this.position = this.position; j = i;
                 }

            }else if(this.timeToExitMode[0]==2){// in sleeping mode
                if(this.timeToExitMode[1]<simulationTime){//and sleeping time elepased
                    // Solve OP-3 to find the next target to visit
                    var ans = this.solveOP3(i);
                    j = ans[0];
                    var rho_ij = ans[1]; 
                    if(j!=i){this.timeToExitMode = [3,simulationTime+rho_ij]};
                }else{
                    // waiting till the end of the predetermined sleeping period    
                    // this.position = this.position; j = i;
                }

            }

            
            if(j==i){
                // stay at i to further reduce the uncertainty till 0
                this.position = this.position;
            }else{
                // need to start moving in the direction of target j
                // rotate
                this.residingTarget = [i,j];
                var headingAngle = atan2P2(targets[i].position,targets[j].position);
                var rotationRequired = headingAngle-this.orientation;
                for(var k = 0; k<this.graphicBaseShape.length ; k++){
                    this.graphicBaseShapeRotated[k] = rotateP2(this.graphicBaseShapeRotated[k], rotationRequired);
                }
                ////print("need to go to j; rotated");
                this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                this.position = plusP2(this.position, this.headingDirectionStep);
                this.orientation = headingAngle;

                if(dataPlotMode){recordSystemState();}// agent Started Moving towards a new target
            }
        }else{// going from T_i to T_j (as this.residingTarget = [T_i, T_j]) 
            var i = this.residingTarget[0];// where we were
            var j = this.residingTarget[1];// where we are heading
            var angle = this.orientation;
            ////print("travelling i to j");
            this.position = plusP2(this.position, this.headingDirectionStep);
            if(distP2(this.position,targets[i].position)>distP2(targets[j].position,targets[i].position)){
                ////print("Stopped at j !!! ")
                this.position = targets[j].position;
                this.residingTarget = [j]; 

                // event driven decision making
                if(this.timeToExitMode[0]==3 && this.timeToExitMode[1]<simulationTime){
                    // solve OP-1 to decide u_i
                    var u_j = this.solveOP1(j);
                    this.timeToExitMode = [1, simulationTime+u_j];
                    //// if u_i = 0 solve OP-3 to find the next target j
                }

                if(dataPlotMode){recordSystemState();}// record agent reached destination target event
            }


        }
    }


    this.solveOP1 = function(i){

        var t_h = Math.min(periodT-simulationTime,timeHorizonForRHC);

        var A_i = targets[i].uncertaintyRate;
        var B_i = this.sensingRate;
        var R_i = targets[i].uncertainty;
        var u_i = R_i/(B_i-A_i);

        return u_i;

    }

    this.solveOP2 = function(i){
        
        var t_h2 = Math.min(periodT-simulationTime,timeHorizonForRHC);
        
        var jArray = []; // set of candidate targets
        var yArray = []; // travel times
        var Abar = targets[i].uncertaintyRate;  
        var Rbar = targets[i].uncertainty;

        for(var k = 0; k<targets[i].neighbors.length; k++){
            var j = targets[i].neighbors[k];
            if( j != i ){
            
                
                // need to find out that no agent is already in or en-route to target j
                var anotherAgentIsComitted = false;
                for(var a = 0; a<agents.length; a++){
                    if(agents[a].residingTarget.length==1){
                        if(agents[a].residingTarget[0]==j){
                            anotherAgentIsComitted = true;
                            break;    
                        }
                    }else if(agents[a].residingTarget.length==2){
                        if(agents[a].residingTarget[1]==j){
                            anotherAgentIsComitted = true;
                            break;
                        }
                    }
                }

                if(!anotherAgentIsComitted){
                    jArray.push(j);
                    var y_ij = targets[i].distancesToNeighbors[k]/this.maxLinearVelocity;
                    yArray.push(y_ij);
                    Abar = Abar + targets[j].uncertaintyRate;    
                    Rbar = Rbar + targets[j].uncertainty;    
                }
                
            }
        }


        var bestDestinationCost = Infinity;
        var bestDestination = i;
        var bestDestinationSleepTime = 0; //if no neighbors, have to wait till next iteration 
        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k];

            // Each coef is multiplied by 2T
            var coefA = (Abar-targets[i].uncertaintyRate);
            var coefB = Abar-this.sensingRate;
            var coefC = (Abar-targets[j].uncertaintyRate);
            var coefD = 2*(Abar-targets[i].uncertaintyRate);
            var coefE = 2*(Abar-targets[i].uncertaintyRate-targets[j].uncertaintyRate);
            var coefF = 2*(Abar-targets[j].uncertaintyRate);
            var coefG = 2*((Rbar-targets[i].uncertainty)+(Abar-targets[i].uncertaintyRate)*y_ij);
            var coefH = 2*((Rbar-targets[i].uncertainty)+Abar*y_ij);
            var coefK = 2*((Rbar-targets[i].uncertainty-targets[j].uncertainty)+(Abar-targets[j].uncertaintyRate)*y_ij);
            var coefL = y_ij*(2*(Rbar-targets[i].uncertainty)+Abar*y_ij);

            var coefs = [coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefL];


            sol1 = this.solveOP2C1(i,j,t_h2,y_ij,coefs);
            sol2 = this.solveOP2C2(i,j,t_h2,y_ij,coefs);
        
            
            if(sol1[0]<sol2[0]){
                if(sol1[0]<bestDestinationCost){
                    // print("1 dom:"+sol1+","+sol2)
                    bestDestinationCost = sol1[0];
                    bestDestination = j;
                    bestDestinationSleepTime = sol1[1];
                }
            }else{
                if(sol2[0]<bestDestinationCost){
                    // print("2 dom:"+sol1+","+sol2)
                    bestDestinationCost = sol2[0];
                    bestDestination = j;
                    bestDestinationSleepTime = sol2[1];
                }
            }

        }

        bestDestinationSleepTime = 0;// temp
        return bestDestinationSleepTime;

    }



    this.solveOP2C1 = function(i,j,t_h2,rho_ij,coefs){

        // transform coeficients
        var coefA = coefs[0];  // A
        var coefB = coefs[1];  // B
        var coefC = coefs[3];  // D
        var coefD = coefs[6];  // G
        var coefE = coefs[7];  // H
        var coefF = coefs[9];  // L

        var coefG = 1;
        var coefH = 1;
        var coefK = rho_ij;

        var coefP = targets[j].uncertaintyRate/(this.sensingRate-targets[j].uncertaintyRate);
        var coefL = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate);
        
        var coefQ = 1;
        var coefM = t_h2-rho_ij;

        var coefN = Infinity;

        var sol = solveBivariateRationalOpt(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN)

        var costVal = sol[0];
        var timeVal = sol[1]; // v_i, u_j, v_j=0

        var costVal = Infinity;
        var timeVal = 0;
        return [costVal, timeVal]; 

    }


    this.solveOP2C2 = function(i,j,t_h2,rho_ij,coefs){

        var alpha = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate);
        var beta = targets[j].uncertaintyRate/(this.sensingRate-targets[j].uncertaintyRate);

        // transform coeficients
        var coefA = coefs[1]*sq(beta) + coefs[3]*beta + coefs[0];  
        var coefB = coefs[2];  
        var coefC = coefs[5]*beta + coefs[4];  
        var coefD = 2*coefs[1]*alpha*beta + coefs[3]*alpha + coefs[7]*beta + coefs[6];
        var coefE = coefs[5]*alpha + coefs[8];
        var coefF = coefs[1]*sq(alpha) + coefs[7]*alpha + coefs[9]; 
        
        var coefG = 1+beta;
        var coefH = 1;
        var coefK = rho_ij+alpha;

        var coefP = 0; 
        var coefL = Infinity;
        
        var coefQ = 1+beta;
        var coefM = t_h2-rho_ij-alpha;

        var coefN = Infinity;

        var sol = solveBivariateRationalOpt(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);

        var costVal = sol[0];
        var timeVal = sol[1]; // v_i, u_j=\lambda_{jo}(v_i), v_j

        var costVal = Infinity;
        var timeVal = 0;
        return [costVal, timeVal]; 

    }





    // OP-3 for ED-RHC

    this.solveOP3 = function(i){

        var t_h3 = Math.min(periodT-simulationTime,timeHorizonForRHC);
        
        var jArray = []; // set of candidate targets
        var yArray = []; // travel times
        var Abar = targets[i].uncertaintyRate;  
        var Rbar = targets[i].uncertainty;

        for(var k = 0; k<targets[i].neighbors.length; k++){
            var j = targets[i].neighbors[k];
            if( j != i ){
            
                
                // need to find out that no agent is already in or en-route to target j
                var anotherAgentIsComitted = false;
                for(var a = 0; a<agents.length; a++){
                    if(agents[a].residingTarget.length==1){
                        if(agents[a].residingTarget[0]==j){
                            anotherAgentIsComitted = true;
                            break;    
                        }
                    }else if(agents[a].residingTarget.length==2){
                        if(agents[a].residingTarget[1]==j){
                            anotherAgentIsComitted = true;
                            break;
                        }
                    }
                }

                if(!anotherAgentIsComitted){
                    jArray.push(j);
                    var y_ij = targets[i].distancesToNeighbors[k]/this.maxLinearVelocity;
                    yArray.push(y_ij);
                    Abar = Abar + targets[j].uncertaintyRate;    
                    Rbar = Rbar + targets[j].uncertainty;    
                }
                
            }
        }


        var bestDestinationCost = Infinity;
        var bestDestination = i;
        var bestDestinationTime = 0;
        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k];

            // Each coef is multiplied by 2T
            var coefA = (Abar-this.sensingRate);
            var coefB = Abar-targets[j].uncertaintyRate;
            var coefC = 2*(Abar-targets[j].uncertaintyRate);
            var coefD = 2*(Rbar+Abar*y_ij);
            var coefE = 2*((Rbar-targets[j].uncertainty)+(Abar-targets[j].uncertaintyRate)*y_ij);
            var coefF = y_ij*(2*Rbar+Abar*y_ij);
            var coefs = [coefA,coefB,coefC,coefD,coefE,coefF]


            var sol1 = this.solveOP3C1(i,j,t_h3,y_ij,Abar,coefs);
            var sol2 = this.solveOP3C2(i,j,t_h3,y_ij,Abar,coefs);
        
            
            if(sol1<sol2){
                if(sol1<bestDestinationCost){
                    // print("1 dom:"+sol1+","+sol2)
                    bestDestinationCost = sol1;
                    bestDestination = j;
                    bestDestinationTime = y_ij;
                }
            }else{
                if(sol2<bestDestinationCost){
                    // print("2 dom:"+sol1+","+sol2)
                    bestDestinationCost = sol2;
                    bestDestination = j;
                    bestDestinationTime = y_ij;
                }
            }
        }

        return [bestDestination, bestDestinationTime];
    
    }


    this.solveOP3C1 = function(i,j,t_h3,rho_ij,Abar,coefs){// u_j = ?, v_j = 0
        var lambda_j0 = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate); 
        var lambda_j = Math.min(lambda_j0,t_h3-rho_ij);    
        
        if(lambda_j<0){// not enough time to visit j
            //print("Error lambda_j:"+lambda_j);
            return Infinity;
        }

        var u_jSharp = Abar*rho_ij/(this.sensingRate-Abar);

        var cost = 0;
        if(u_jSharp <= lambda_j && Abar < this.sensingRate){
            cost = evalCostOP3(lambda_j,0,rho_ij,coefs); //u_j = lambda_j
            //print("testOP3C1: i="+i+", j="+j+", Cost"+cost+"<"+evalCostOP3(0,0,coefs));
        }else{
            cost = evalCostOP3(0,0,rho_ij,coefs); // u_j = 0
            //print("testOP3C1: i="+i+", j="+j+", Cost"+cost+"<"+evalCostOP3(lambda_j,0,coefs));
        }
        
        cost1 = evalCostOP3(lambda_j,0,rho_ij,coefs);
        cost2 = evalCostOP3(0,0,rho_ij,coefs);     
        ////print("testOP3C1: i="+i+", j="+j+", Cost1:"+cost1.toFixed(3)+", Cost2:"+cost2.toFixed(3)+", Cost:"+cost.toFixed(3)) 

        if(min(cost1,cost2)!=cost){
            print("Error: u_jSharp: "+u_jSharp+", lambda_j: "+lambda_j)
            print("testOP3C1: i="+i+", j="+j+", Cost1:"+cost1.toFixed(3)+", Cost2:"+cost2.toFixed(3)+", Cost:"+cost.toFixed(3)) 
        }
        return cost;

    }

    this.solveOP3C2 = function(i,j,t_h3,rho_ij,Abar,coefs){ // u_j = \lambda_j0, v_j = ?
        var lambda_j0 = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate); 
        var mu_j = t_h3-(rho_ij+lambda_j0)

        if(mu_j<0){// this means it should belong to class 1
            //print("Error: mu_j: "+mu_j)
            return Infinity;
        }
        
        var temp1 = (this.sensingRate-targets[j].uncertaintyRate)*sq(rho_ij+lambda_j0)-this.sensingRate*sq(rho_ij);
        var v_jSharp = Math.sqrt((temp1)/(Abar-targets[j].uncertaintyRate))-(rho_ij+lambda_j0);


        var temp2 = this.sensingRate*(1-sq(rho_ij)/sq(rho_ij+lambda_j0))
        
        var cost = 0;
        if(Abar >= temp2){
            cost = evalCostOP3(lambda_j0,0,rho_ij,coefs); //v_j = 0
        }else if (v_jSharp >= mu_j){
            cost = evalCostOP3(lambda_j0,mu_j,rho_ij,coefs); // v_j = mu_j
        }else{
            cost = evalCostOP3(lambda_j0,v_jSharp,rho_ij,coefs);
        }


        // no need
        // cost1 = evalCostOP3(lambda_j0,0,rho_ij,coefs);
        // cost2 = evalCostOP3(lambda_j0,mu_j,rho_ij,coefs);
        // cost3 = evalCostOP3(lambda_j0,v_jSharp,rho_ij,coefs);
        // ////print("testOP3C2: i="+i+", j="+j+", Cost1:"+cost1.toFixed(3)+", Cost2:"+cost2.toFixed(3)+", Cost3:"+cost3.toFixed(3)+", Cost:"+cost.toFixed(3)) 

        // if(min(cost1,cost2,cost3)!=cost){ // this happens only when v_jShapr is negative! hens theory is correct!!!!
        //     //print("Error: ")
        //     print("v_jSharp: "+v_jSharp)
        //     print("testOP3C2: i="+i+", j="+j+", Cost1:"+cost1.toFixed(3)+", Cost2:"+cost2.toFixed(3)+", Cost3:"+cost3.toFixed(3)+", Cost:"+cost.toFixed(3))
        // }

        return cost;
    }





    this.IPAComputationEventD1 = function(targetID){
        var i = targetID;
        var a = this.id;

        // event time sensitivity - update : eventTimeSensitivity[z][p][q]
        for(var z = 0; z < agents.length; z++){
            for(var p = 0; p < targets.length; p++){
                for(var q = 0; q < targets.length; q++){
                    if ( (p==i && q==i) && z==a ){
                        eventTimeSensitivity[z][p][q] = -1*(-1+targets[i].sensitivityOfUncertainty[z][p][q])/(targets[i].uncertaintyRate - this.getNeighborhoodSensingRate());
                    }else{
                        eventTimeSensitivity[z][p][q] = -1*(targets[i].sensitivityOfUncertainty[z][p][q])/(targets[i].uncertaintyRate - this.getNeighborhoodSensingRate());
                    }
                    targets[i].sensitivityOfUncertainty[z][p][q] = targets[i].sensitivityOfUncertainty[z][p][q] - this.sensingRate*eventTimeSensitivity[z][p][q];
                }
            }
        }          
        // end event time sensitivity

    }

    this.IPAComputationEventA1 = function(currentTargetID,nextTargetID){
        var i = currentTargetID; 
        var j = nextTargetID;
        var a = this.id;

        // event time sensitivity - update : eventTimeSensitivity[z][p][q]
        eventTimeSensitivity = [...this.savedEventTimeSensitivity];
        for(var z = 0; z<agents.length; z++){
            for(var p = 0; p < targets.length; p++){
                for(var q = 0; q < targets.length; q++){
                    ////eventTimeSensitivity[z][p][q] = eventTimeSensitivity[z][p][q]; // no need to change
                    // if ((p==i && q==i) && z==a ){
                    //     eventTimeSensitivity[z][p][q] = -1*(-1+targets[i].sensitivityOfUncertainty[z][p][q])/(targets[i].uncertaintyRate - targets[i].getNetAgentSensingRate());
                    // }else{
                    //     eventTimeSensitivity[z][p][q] = -1*(targets[i].sensitivityOfUncertainty[z][p][q])/(targets[i].uncertaintyRate - targets[i].getNetAgentSensingRate());
                    // }

                    targets[j].sensitivityOfUncertainty[z][p][q] = targets[j].sensitivityOfUncertainty[z][p][q] + this.sensingRate*eventTimeSensitivity[z][p][q];
                }
            }
        }     
    }

    this.IPAComputationEventD2 = function(currentTargetID,nextTargetID){
        var i = currentTargetID;
        var j = nextTargetID;
        var a = this.id;

        // event time sensitivity - update : eventTimeSensitivity[z][p][q]
        for(var z = 0; z<agents.length; z++){
            for(var p = 0; p < targets.length; p++){
                for(var q = 0; q < targets.length; q++){
                    if ((p==i && q==j) && z==a ){
                        eventTimeSensitivity[z][p][q] = -1*(-1+targets[j].sensitivityOfUncertainty[z][p][q])/(targets[j].uncertaintyRate - targets[j].getNetAgentSensingRate());
                        // if(abs(
                        //     eventTimeSensitivity[z][p][q]) > 1000001){
                        //     eventTimeSensitivity[z][p][q]=0;
                        //     print("Error lies here 1!!!")
                        //     print(targets[j].uncertaintyRate);
                        //     print(targets[j].getNetAgentSensingRate());
                        //     print(targets[j].sensitivityOfUncertainty[z][p][q]);
                        // }
                    }else{
                        eventTimeSensitivity[z][p][q] = -1*(targets[j].sensitivityOfUncertainty[z][p][q])/(targets[j].uncertaintyRate - targets[j].getNetAgentSensingRate());
                        // if(abs(
                        //     eventTimeSensitivity[z][p][q]) > 1000001){
                        //     eventTimeSensitivity[z][p][q]=0;
                        //     print("Error lies her 2!!!")
                        //     print(targets[j].uncertaintyRate);
                        //     print(targets[j].getNetAgentSensingRate());
                        //     print(targets[j].sensitivityOfUncertainty[z][p][q]);
                        // }
                    }
                    
                    targets[i].sensitivityOfUncertainty[z][p][q] = targets[i].sensitivityOfUncertainty[z][p][q] - this.sensingRate*eventTimeSensitivity[z][p][q];
                }
            }
        }          
        // end event time sensitivity

    }

    this.IPAComputationEventD3_1 = function(currentTargetID,nextTargetID){
        var i = currentTargetID; 
        var j = nextTargetID;
        var a = this.id;

        // event time sensitivity - update : eventTimeSensitivity[z][p][q]
        for(var z = 0; z<agents.length; z++){
            for(var p = 0; p < targets.length; p++){
                for(var q = 0; q < targets.length; q++){
                    if ((p==i && q==j) && z==a ){
                        eventTimeSensitivity[z][p][q] = -1*(-1+targets[j].sensitivityOfUncertainty[z][p][q])/(targets[j].uncertaintyRate - targets[j].getNetAgentSensingRate());
                    }else{
                        eventTimeSensitivity[z][p][q] = -1*(targets[j].sensitivityOfUncertainty[z][p][q])/(targets[j].uncertaintyRate - targets[j].getNetAgentSensingRate());
                    }
                    targets[i].sensitivityOfUncertainty[z][p][q] = -1*(targets[i].uncertaintyRate - this.getImmediateNeighborsSensingRate())*eventTimeSensitivity[z][p][q];
                }
            }
        }          
        // end event time sensitivity

    }

    this.IPAComputationEventD3_2 = function(currentTargetID,nextTargetID){
        var i = currentTargetID; 
        var j = nextTargetID;
        var a = this.id;

        // event time sensitivity - update : eventTimeSensitivity[z][p][q]
        for(var z = 0; z<agents.length; z++){
            for(var p = 0; p < targets.length; p++){
                for(var q = 0; q < targets.length; q++){
                    if ((p==i && q==j) && z==a ){
                        eventTimeSensitivity[z][p][q] = -1*(-1+targets[j].sensitivityOfUncertainty[z][p][q])/(targets[j].uncertaintyRate - targets[j].getNetAgentSensingRate());
                    }else{
                        eventTimeSensitivity[z][p][q] = -1*(targets[j].sensitivityOfUncertainty[z][p][q])/(targets[j].uncertaintyRate - targets[j].getNetAgentSensingRate());
                    }
                    targets[i].sensitivityOfUncertainty[z][p][q] = 0; 
                }
            }
        }          
        // end event time sensitivity

    }


    this.IPAComputationEventA2 = function(currentTargetID,nextTargetID){
        var i = currentTargetID; 
        var j = nextTargetID;
        var a = this.id;

        // event time sensitivity - update : eventTimeSensitivity[z][p][q]
        eventTimeSensitivity = [...this.savedEventTimeSensitivity];
        for(var z = 0; z<agents.length; z++){
            for(var p = 0; p < targets.length; p++){
                for(var q = 0; q < targets.length; q++){
                    ////eventTimeSensitivity[z][p][q] = eventTimeSensitivity[z][p][q]; // no need to change
                    // if ((p==i && q==j) && z==a ){
                    //     eventTimeSensitivity[z][p][q] = -1*(-1+targets[j].sensitivityOfUncertainty[z][p][q])/(targets[j].uncertaintyRate - targets[j].getNetAgentSensingRate());
                    // }else{
                    //     eventTimeSensitivity[z][p][q] = -1*(targets[j].sensitivityOfUncertainty[z][p][q])/(targets[j].uncertaintyRate - targets[j].getNetAgentSensingRate());
                    // }
                    ////print("i="+i+", j="+j+", a="+a+"; z="+z+", p="+p+", q="+q);
                    targets[j].sensitivityOfUncertainty[z][p][q] = targets[j].sensitivityOfUncertainty[z][p][q] + this.sensingRate*eventTimeSensitivity[z][p][q];
                }
            }
        }     
    }


    this.IPAComputationEventE3 = function(currentTargetID){
        var i = currentTargetID; 
        var a = this.id;

        // event time sensitivity - update : eventTimeSensitivity[z][p][q]
        for(var z = 0; z<agents.length; z++){
            for(var p = 0; p < targets.length; p++){
                for(var q = 0; q < targets.length; q++){
                    eventTimeSensitivity[z][p][q] = -1*(targets[i].sensitivityOfUncertainty[z][p][q])/(targets[i].uncertaintyRate - this.getNeighborhoodSensingRate());
                    targets[i].sensitivityOfUncertainty[z][p][q] = 0;
                }
            }
        }

    }

    this.IPAComputationEventE4 = function(currentTargetID){
        var i = currentTargetID;

        // event time sensitivity - update : eventTimeSensitivity[z][p][q]
        for(var z = 0; z<agents.length; z++){
            for(var p = 0; p < targets.length; p++){
                for(var q = 0; q < targets.length; q++){
                    eventTimeSensitivity[z][p][q] = 0;
                    targets[i].sensitivityOfUncertainty[z][p][q] = targets[i].sensitivityOfUncertainty[z][p][q];
                }
            }
        }
                     
    }




    this.getImmediateNeighbors = function(){
        var i = this.residingTarget[0];
        var result = [];
        for(var j =0 ;j<agents.length; j++){
            if(targets[i].residingAgents[j] == true && j!=this.id){
                result.push(j);
            }
        }
        return result;
    }

    this.getImmediateNeighborsSensingRate = function(){
        var neighbors = this.getImmediateNeighbors();
        var sumVal = 0;
        for(var j =0; j<neighbors.length; j++){
            sumVal = sumVal + agents[neighbors[j]].sensingRate;
        }
        return sumVal;
    }

    this.getNeighborhoodSensingRate = function(){
        var neighbors = this.getImmediateNeighbors();
        var sumVal = this.sensingRate;
        for(var j =0; j<neighbors.length; j++){
            sumVal = sumVal + agents[neighbors[j]].sensingRate;
        }
        return sumVal;
    }



    this.findNextTarget = function(currentTargetIndex){// to find j such that R_j(t)<theta_{ij}^a
        var i = currentTargetIndex;
        var jArray = []; // set of candidate targets
        for(var j = 0; j<targets.length; j++){
            if( j != i && targets[j].uncertainty > this.threshold[i][j]){
                //print(paths[getPathID(i,j)].isPermanent)
                if(paths[getPathID(i,j)].isPermenent){
                    jArray.push(j);        
                }
                
            }
        }

        if(jArray.length==0){// no need to go to a neighbor
            return i;
        }else if(jArray.length == 1){
            ////print("Goto j");
            return jArray[0];
        }else{
            ////print("Need a tie breaker");

            // minimum distance tie breaker
            var minDistanceTargetIndex = jArray[0];
            var minDistanceFound = 10000;
            for(var k = 0; k < jArray.length; k++){
                var j = jArray[k];
                var distance_ij = distP2(targets[i].position,targets[j].position);
                if(distance_ij<minDistanceFound){
                    minDistanceFound = distance_ij;
                    minDistanceTargetIndex = j;
                }
            }
            //return minDistanceTargetIndex;
            // end minimum distance tie breaker

            // maximum uncertainty tie breaker?
            var maxUncertaintyTargetIndex = jArray[0];
            var maxUncertaintyFound = 0;
            for(var k = 0; k < jArray.length; k++){
                var j = jArray[k];
                var uncertaintyLevel = targets[j].uncertainty;
                if(uncertaintyLevel>maxUncertaintyFound){
                    maxUncertaintyFound = uncertaintyLevel;
                    maxUncertaintyTargetIndex = j;
                }
            }
           //return maxUncertaintyTargetIndex;
            // end maximum uncertainty tie breaker?

            if(targetPrioritizationPolicy==3){
                if(Math.random()>0.5){
                    return minDistanceTargetIndex;
                }else{
                    return maxUncertaintyTargetIndex;
                }    
            }else if(targetPrioritizationPolicy==2){
                return maxUncertaintyTargetIndex;
            }else{
                return minDistanceTargetIndex;
            }
          
        }

    }



    this.findNextTargetTwoStepsAheadRHC = function(currentTargetIndex){
        //print("here")
        var i = currentTargetIndex;
        var jIndArray = []; // set of candidate targets
        var yArray = []; // travel times

        for(var l = 0; l<targets[i].neighbors.length; l++){

            var j = targets[i].neighbors[l];
            if( j != i ){
                
                // need to find out that no agent is already in or en-route to target j
                var anotherAgentIsComitted = false;
                for(var a = 0; a<agents.length; a++){
                    if(agents[a].residingTarget.length==1){
                        if(agents[a].residingTarget[0]==j){
                            anotherAgentIsComitted = true;
                            break;    
                        }
                    }else if(agents[a].residingTarget.length==2){
                        if(agents[a].residingTarget[1]==j){
                            anotherAgentIsComitted = true;
                            break;
                        }
                    }
                }

                if(!anotherAgentIsComitted){
                    jIndArray.push(l);
                    var y_ij = targets[i].distancesToNeighbors[l]/this.maxLinearVelocity;
                    yArray.push(y_ij);    
                }
                                  
                
                
            }
        }


        if(jIndArray.length==0){// no need to go to a neighbor
            return i;
        }else{
            //print("Agent "+(this.id+1)+" at Target "+(i+1)+" deciding between: "+jIndArray);
            // print("Agent "+(this.id+1)+" deciding between: ");
            //print(jArray);
            //print(yArray);
        }

        
        // now we need to execute the one step (ahead) greedy selection.
        ////var maxGain = 0;
        var minGain = Infinity;
        var minGainDestinationTarget = i;
        var bestPath = [];

        for(var l = 0; l < jIndArray.length; l++){
            
            var j = targets[i].neighbors[jIndArray[l]];
            var y_ij = yArray[l];
            var t_h = Math.min(periodT-simulationTime,timeHorizonForRHC);

            var A_i = targets[i].uncertaintyRate;
            var A_j = targets[j].uncertaintyRate;
            var B_j = this.sensingRate; 
            var B_i = B_j;
            var R_j0 = targets[j].uncertainty;

            var tau_jHat = (R_j0+A_j*y_ij)/(B_j-A_j);
        
            var tau_jBar = Math.min(tau_jHat,t_h-y_ij);
            var delta_jBar = t_h-(y_ij+tau_jHat);

            var tau_j; //Optimum values
            var delta_j;


            for(var m = 0; m < targets[i].neighbors2[jIndArray[l]].length; m++){
                
                var k = targets[i].neighbors2[jIndArray[l]][m];
                var y_jk = targets[i].distancesToNeighbors2[jIndArray[l]][m]/this.maxLinearVelocity;

                var A_k = targets[k].uncertaintyRate;
                var B_k = B_i;
                var R_k0 = targets[k].uncertainty;

                var tau_kHat0 = (R_k0+A_k*(y_ij+tau_jHat+y_jk))/(B_k-A_k);
                
                // if the main ass does not hold or when g_4 or g_2 conditionas are activated we need to follow the one step method!
                if(t_h>(y_ij+tau_jHat+y_jk+tau_kHat0) && R_j0 > A_i*y_jk && (B_i-A_i)*y_jk-A_i*y_ij>0 ){// can go two steps ahead on this path
                    // above condition valid for loops as well
                    var tau_k;
                    var delta_k;
                    
                    var delta_kBarHat = t_h-(y_ij+tau_jHat+y_jk+tau_kHat0);
                    var delta_jBarHat = (B_k-A_k)*delta_kBarHat/B_k;

                    if(k!=i){// non-loops case: delta_j and delta_k
                        // h-switch functions  
                        var sigma_1  = B_i*y_jk/(B_i-A_i) - t_h;
                        var tempVal1 = sq(B_i)/(A_i*(B_i-A_i));
                        var tempVal2 = -A_i*(2*B_i-A_i)*(R_j0+B_i*y_ij)/(B_i*(B_i-A_i)) - A_i*(B_i-A_i)*sigma_1/B_i;
                        
                        if(sigma_1*A_i - tempVal1 < R_k0 && R_k0 < A_i*sigma_1){
                            tau_j = tau_jHat;
                            delta_j = delta_jBarHat;
                            tau_k = (R_k0+A_k*(y_ij+tau_j+delta_j+y_jk))/(B_k-A_k);
                            delta_k = 0;
                        }else if(tempVal2 < R_k0){
                            tau_j = tau_jHat;
                            delta_j = 0;
                            tau_k = tau_kHat0;
                            delta_k = delta_kBarHat; 
                        }else{
                            tau_j = 0;
                            delta_j = 0;
                            tau_k = 0;
                            delta_k = 0; 
                        }

                        // J_ijk
                        var gainVal = tau_j*( (A_i-B_j)*tau_j + (2*(A_i-A_j))*delta_j + (2*(A_i-B_j))*tau_k + (2*(A_i-A_k-B_j))*delta_k + (2*(A_i*y_ij+A_i*y_jk-B_j*y_jk))) + 
                                      delta_j*( (A_i-A_j)*delta_j + (2*(A_i-A_j))*tau_k + (2*(A_i-A_j-A_k))*delta_k + (2*(A_i*y_ij-R_j0+A_i*y_jk-A_j*y_ij-A_j*y_jk))) + 
                                      tau_k*( (A_i-B_k)*tau_k + (2*(A_i-A_k))*delta_k + (2*A_i*(y_ij+y_jk))  )+
                                      delta_k*( (A_i-A_k)*delta_k + (2*(A_i*y_ij-R_k0+A_i*y_jk-A_k*y_ij-A_k*y_jk))) +
                                      (A_i*sq(y_ij+y_jk));  
                           
                        if(gainVal<0 && gainVal<minGain){
                            minGain = gainVal;
                            minGainDestinationTarget = j;
                            bestPath = [i,j,k]
                            ////print("Agent "+(this.id+1)+" at Target "+(i+1)+" decided: ["+(i+1)+","+(j+1)+","+(k+1)+"] path");
                        }

                    }else{// loops case: delta_j and delta_k
                        // q-switch functions
                        // k = i for this case
                        var sigma_2 = A_i*B_i/(A_i*B_i+A_j*(B_i-A_i))
                        var sigma_3 = B_i*y_ij/(B_i-A_i) - t_h;
                        var sigma_4 = B_i*(3*B_i-2*A_i)*y_ij/(2*B_i-A_i) - sq((B_i-A_i))*t_h/(2*B_i-A_i) 
                        
                        if(sigma_3>0 && R_j0>sq(A_i/B_i)*(B_i-A_i)*sigma_3){
                            tau_j = tau_jHat;
                            delta_j = delta_jBarHat;
                            tau_k = (R_k0+A_k*(y_ij+tau_j+delta_j+y_jk))/(B_k-A_k);
                            delta_k = 0;
                        }else if(sigma_3<0){
                            tau_j = tau_jHat;
                            delta_j = t_h-y_ij-tau_jHat-sigma_2*(t_h+y_ij);
                            tau_k = (R_k0+A_k*(y_ij+tau_j+delta_j+y_jk))/(B_k-A_k);
                            delta_k = t_h - sigma_2*A_j*(t_h+y_ij)/A_i
                        }else if(R_j0<sigma_4){
                            tau_j = tau_jHat;
                            delta_j = 0;
                            tau_k = tau_kHat0;
                            delta_k = delta_kBarHat; 
                        }else{
                            tau_j = 0;
                            delta_j = 0;
                            tau_k = 0;
                            delta_k = 0; 
                        }

                        // J_iji
                        var gainVal = tau_j*( (A_i-B_j)*tau_j + (2*(A_i-A_j))*delta_j + (A_i-B_j)*tau_k + (-2*B_j)*delta_k + (2*y_ij*(2*A_i-B_j))) + 
                                      delta_j*( (A_i-A_j)*delta_j + (2*(A_i-A_j))*tau_k + (-2*A_j)*delta_k + (4*(A_i-A_j)*y_ij-2*R_j0)) + 
                                      tau_k*( (A_i-B_i)*tau_k + (4*A_i*y_ij)) + 
                                      (4*A_i*sq(y_ij)); 

                        //gainVal = gainVal + t_h*(2*R_k0+A_k*t_h);
                           
                        if(gainVal<0 && gainVal<minGain){
                            minGain = gainVal;
                            minGainDestinationTarget = j;
                            bestPath = [i,j,k]
                            ////print("Agent "+(this.id+1)+" at Target "+(i+1)+" decided: ["+(i+1)+","+(j+1)+","+(k+1)+"] path");
                        }

                    }

                }else{// not able to go two steps on this path
                    
                    // no use of 'k' here.
                    var tempCond1 = 2*A_i*y_ij/(B_i-A_i);
                    if(tau_jBar<tempCond1){
                        // no point in going to j !
                        tau_j = 0;
                        delta_j = 0;

                    }else{
                        tau_j = tau_jBar
                        if(A_i>A_j){
                            if(R_j0<(A_i-A_j)*(y_ij+tau_jHat)){
                                delta_j = 0;
                            }else if(R_j0<(A_i-A_j)*t_h){
                                delta_j = (R_j0/(A_i-A_j))-(y_ij+tau_jHat);
                            }else{
                                delta_j = delta_jBar;
                            }
                        }else{
                            delta_j = delta_jBar;
                        }

                        var gainVal = A_i*sq(y_ij) + 2*tau_j*y_ij*A_i - 2*delta_j*((A_j-A_i)*y_ij+R_j0) - 2*delta_j*tau_j*(A_j-A_i) - (B_j-A_i)*sq(tau_j) - (A_j-A_i)*sq(delta_j); //theoretically, this needs to be divided by (2*periodT);
                        // Gain considering cost of k
                        gainVal = gainVal + t_h*(2*R_k0+A_k*t_h);
                            
                        
                        if(gainVal<0 && gainVal<minGain){
                            minGain = gainVal;
                            minGainDestinationTarget = j;
                            bestPath = [i,j];
                            ////print("Agent "+(this.id+1)+" at Target "+(i+1)+" decided: ["+(i+1)+","+(j+1)+"] path");
                        }
                    }

                }

            }
            
        }
        //if(bestPath.length>0 && bestPath[0]!=bestPath[bestPath.length-1]){
        if(bestPath.length>0 ){
            print("Best Path: ["+bestPath+"], Cost increment: "+minGain/(2*periodT));
        }
        return minGainDestinationTarget;
    }



    this.findNextTargetOneStepAheadRHC = function(currentTargetIndex){

        var i = currentTargetIndex;
        var jArray = []; // set of candidate targets
        var yArray = []; // travel times

        for(var k = 0; k<targets[i].neighbors.length; k++){

            var j = targets[i].neighbors[k];
            if( j != i ){
                
                
                // need to find out that no agent is already in or en-route to target j
                var anotherAgentIsComitted = false;
                for(var a = 0; a<agents.length; a++){
                    if(agents[a].residingTarget.length==1){
                        if(agents[a].residingTarget[0]==j){
                            anotherAgentIsComitted = true;
                            break;    
                        }
                    }else if(agents[a].residingTarget.length==2){
                        if(agents[a].residingTarget[1]==j){
                            anotherAgentIsComitted = true;
                            break;
                        }
                    }
                }

                if(!anotherAgentIsComitted){
                    jArray.push(j);
                    var y_ij = targets[i].distancesToNeighbors[k]/this.maxLinearVelocity;
                    yArray.push(y_ij);    
                }              
                
            }
        }


        if(jArray.length==0){// no need to go to a neighbor
            return i;
        }else{
            //// print("Agent "+(this.id+1)+" at Target "+(i+1)+" deciding between: "+jArray);
        }

        
        // now we need to execute the one step (ahead) greedy selection.
        ////var maxGain = 0;
        var minGain = Infinity;
        var minGainDestinationTarget = i;

        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k];
            var t_h = Math.min(periodT-simulationTime,timeHorizonForRHC);

            var A_i = targets[i].uncertaintyRate;
            var A_j = targets[j].uncertaintyRate;
            var B_j = this.sensingRate; 
            var B_i = B_j;
            var R_j0 = targets[j].uncertainty;

            var tau_jHat = (R_j0+A_j*y_ij)/(B_j-A_j);
        
            var tau_jBar = Math.min(tau_jHat,t_h-y_ij);
            var delta_jBar = t_h-(y_ij+tau_jHat);

            var tau_j; //Optimum values
            var delta_j;

            var tempCond1 = 2*A_i*y_ij/(B_i-A_i);
            if(tau_jBar<tempCond1){
                // no point in going to j !
                //// print("Omit target "+(j))
                tau_j = 0;
                delta_j = 0;
            }else{
                tau_j = tau_jBar
                if(A_i>A_j){
                    if(R_j0<(A_i-A_j)*(y_ij+tau_jHat)){
                        delta_j = 0;
                    }else if(R_j0<(A_i-A_j)*t_h){
                        delta_j = (R_j0/(A_i-A_j))-(y_ij+tau_jHat);
                    }else{
                        delta_j = delta_jBar;
                    }
                }else{
                    delta_j = delta_jBar;
                }

                var gainVal = A_i*sq(y_ij) + 2*tau_j*y_ij*A_i - 2*delta_j*((A_j-A_i)*y_ij+R_j0) - 2*delta_j*tau_j*(A_j-A_i) - (B_j-A_i)*sq(tau_j) - (A_j-A_i)*sq(delta_j); //theoretically, this needs to be divided by (2*periodT);
                
                
                if(gainVal<0 && gainVal<minGain){
                    minGain = gainVal;
                    minGainDestinationTarget = j;
                }
            }

                        
            
            
        }

        if(minGainDestinationTarget!=i){
            print("Agent "+(this.id+1)+" at Target "+(i+1)+" deciding between: "+plusOneToArray(jArray)+" decided: Target "+(minGainDestinationTarget+1));
        }
        return minGainDestinationTarget;

    }

    
    // first attamept working version
    // this.findNextTargetOneStepGreedy = function(currentTargetIndex){

    //     var i = currentTargetIndex;
    //     var jArray = []; // set of candidate targets
    //     var yArray = []; // travel times

    //     for(var k = 0; k<targets[i].neighbors.length; k++){

    //         var j = targets[i].neighbors[k];
    //         if( j != i ){
                
    //             if(targets[j].residingAgents.length==0){ 
                    
    //                 // need to find out that no agent is en-route to target j
    //                 var anotherAgentIsComitted = false;
    //                 for(var a = 0; a<agents.length; a++){
    //                     if(agents[a].residingTarget[1]==j){
    //                         anotherAgentIsComitted = true;
    //                         break;
    //                     }
    //                 }

    //                 if(!anotherAgentIsComitted){
    //                     jArray.push(j);
    //                     var y_ij = targets[i].distancesToNeighbors[k]/this.maxLinearVelocity;
    //                     yArray.push(y_ij);    
    //                 }
                                  
    //             }
                
    //         }
    //     }


    //     if(jArray.length==0 || targets[i].uncertainty>0){// no need to go to a neighbor
    //         return i;
    //     }else if(jArray.length>0 && targets[i].uncertainty==0){
    //         print("Agent "+(this.id+1)+" deciding between: ");
    //         print(jArray);
    //         //print(yArray);
    //     }

        
    //     // now we need to execute the one step (ahead) greedy selection.
    //     ////var maxGain = 0;
    //     var minGain = Infinity;
    //     var minGainDestinationTarget = i;

    //     for(var k = 0; k < jArray.length; k++){
            
    //         var j = jArray[k];
    //         var y_ij = yArray[k];
            
    //         var A_i = targets[i].uncertaintyRate;
    //         var A_j = targets[j].uncertaintyRate;
    //         var B_j = this.sensingRate; 
    //         var R_j0 = targets[j].uncertainty;

    //         var tau_jHat = (R_j0+A_j*y_ij)/(B_j-A_j);
            
    //         var tempCond = 2*(B_j-A_j)*(A_i*y_ij)/(B_j-A_i)-A_j*y_ij;
    //         if(y_ij>(periodT-simulationTime)|| R_j0<tempCond){
    //             // no point in going to this destination
    //         }

    //         else if((periodT-simulationTime)<tau_jHat){
    //             var tau_j = (periodT-simulationTime);
    //             var gainVal;

    //             if(RHCMethod==1){
    //                 gainVal = A_i*sq(y_ij) + 2*tau_j*y_ij*A_i - (B_j-A_i)*sq(tau_j); //theoretically, this needs to be divided by (2*periodT);
    //                 ////gainVal = (tau_j*(B_j-A_i)-y_ij*A_i)/periodT;
    //             }else{
    //                 gainVal = (A_i*sq(y_ij) + 2*tau_j*y_ij*A_i-(B_j-A_i)*sq(tau_j))/(y_ij+tau_j);
    //                 ////gainVal = (tau_j*(B_j-A_i)-y_ij*A_i)/(tau_j*periodT)
    //             }

    //             if(gainVal<minGain){
    //                 minGain = gainVal;
    //                 minGainDestinationTarget = j;
    //             }
    //             // jkjkvbsvbsdvbioas
    //         }else{
    //             var tau_j = tau_jHat;
    //             var delta_j = (periodT-simulationTime)-(y_ij+tau_j);
                
    //             var gainVal;
    //             if(RHCMethod==1){
    //                 gainVal = A_i*sq(y_ij) + 2*tau_jHat*y_ij*A_i - 2*delta_j*((A_j-A_i)*y_ij+R_j0) - 2*delta_j*tau_jHat*(A_j-A_i) - (B_j-A_i)*sq(tau_jHat) - (A_j-A_i)*sq(delta_j); //theoretically, this needs to be divided by (2*periodT);
    //                 ////gainVal = (tau_j*(B_j-A_i)+delta_j*(A_j-A_i)-y_ij*A_i)/periodT;
    //             }else{
    //                 gainVal = (A_i*sq(y_ij) + 2*tau_jHat*y_ij*A_i - 2*delta_j*((A_j-A_i)*y_ij+R_j0) - 2*delta_j*tau_jHat*(A_j-A_i) - (B_j-A_i)*sq(tau_jHat) - (A_j-A_i)*sq(delta_j))/(y_ij+tau_jHat+delta_j);
    //                 ////gainVal = (tau_j*(B_j-A_i)+delta_j*(A_j-A_i)-y_ij*A_i)/((tau_j+delta_j)*periodT);
    //             }
                
    //             if(gainVal<minGain){
    //                 minGain = gainVal;
    //                 minGainDestinationTarget = j;
    //             }
    //         }
    //     }

    //     return minGainDestinationTarget;

    // }


    this.findNextTargetSplitBoost = function(currentTargetIndex){// to find j such that R_j(t)<theta_{ij}^a
        
        var i = currentTargetIndex;
        var nextTargetOfThis = this.findNextTarget(i);

        //// split begin
        var otherAgentsReadyToLeaveForSameTarget = [];
        for(var a = 0; a<agents.length; a++){
            if(agents[a].residingTarget[0]==i && a<this.id){// target is being shared with a dominent neighbor 
                var j = agents[a].findNextTarget(i);
                if(targets[i].uncertainty < agents[a].threshold[i][i] && nextTargetOfThis==j){
                    otherAgentsReadyToLeaveForSameTarget.push(a);
                }
            }
        }
        // if(otherAgentsReadyToLeaveForSameTarget.length>0){
        //     //print("This: "+this.id+" , followed by: "+otherAgentsReadyToLeaveForSameTarget);    
        // }
        //print("Length = "+residingAgentsInTarget.length);
        //// split end


        
        var jArray = []; // set of candidate targets
        for(var j = 0; j<targets.length; j++){
            if( j != i && targets[j].uncertainty > this.threshold[i][j]){
                //print(paths[getPathID(i,j)].isPermanent)
                if(paths[getPathID(i,j)].isPermenent){
                    if(nextTargetOfThis==j && otherAgentsReadyToLeaveForSameTarget.length>0){
                        this.threshold[i][j] = boostingCoefficientAlpha*targets[j].uncertainty + (1-boostingCoefficientAlpha)*this.threshold[i][j];// +  boostingCoefficientAlpha;
                         
                        // cannot go to j !!!!
                    }else{
                        jArray.push(j);    // j is an available option         
                    }
                    
                }
                
            }
        }

        if(jArray.length==0){// no need to go to a neighbor
            return i;
        }else if(jArray.length == 1){
            ////print("Goto j");
            return jArray[0];
        }else{
            ////print("Need a tie breaker");

            // minimum distance tie breaker
            var minDistanceTargetIndex = jArray[0];
            var minDistanceFound = 10000;
            for(var k = 0; k < jArray.length; k++){
                var j = jArray[k];
                var distance_ij = distP2(targets[i].position,targets[j].position);
                if(distance_ij<minDistanceFound){
                    minDistanceFound = distance_ij;
                    minDistanceTargetIndex = j;
                }
            }
            //return minDistanceTargetIndex;
            // end minimum distance tie breaker

            // maximum uncertainty tie breaker?
            var maxUncertaintyTargetIndex = jArray[0];
            var maxUncertaintyFound = 0;
            for(var k = 0; k < jArray.length; k++){
                var j = jArray[k];
                var uncertaintyLevel = targets[j].uncertainty;
                if(uncertaintyLevel>maxUncertaintyFound){
                    maxUncertaintyFound = uncertaintyLevel;
                    maxUncertaintyTargetIndex = j;
                }
            }
           //return maxUncertaintyTargetIndex;
            // end maximum uncertainty tie breaker?

            if(targetPrioritizationPolicy==3){
                if(Math.random()>0.5){
                    return minDistanceTargetIndex;
                }else{
                    return maxUncertaintyTargetIndex;
                }    
            }else if(targetPrioritizationPolicy==2){
                return maxUncertaintyTargetIndex;
            }else{
                return minDistanceTargetIndex;
            }
          
        }

    }



    this.findProbableNextTarget = function(currentTargetIndex){// to find j such that R_j(t)<theta_{ij}^a
        var i = currentTargetIndex;
        
        var nextTarget = i;
        var nextTargetValue = targets[i].uncertainty; // 0

        for(var j = 0; j<targets.length; j++){
            var val = targets[j].uncertainty;
            var pID = getPathID(i,j);
            if( j != i && nextTargetValue>=val && paths[pID].isPermanent){
                nextTargetValue = val;
                nextTarget = j;    
            }
        }

        return nextTarget;
    }


    this.assignToTheTarget = function(targetID){
        this.position = targets[targetID].position;
        this.residingTarget = [targetID];
        this.initialResidingTarget = targetID;
    }


    this.backSearchForCycles = function(){
        if(this.visitedTargetList.length<4 || this.unvisitedTargetsInfo.length<4){ // nothing to search for
            //return -1;
        }else{
            var maxCycleLength = targets.length; // typically depends on B_i/A_i
            var L = this.visitedTargetList.length;
            var blockSizeFound = false; //redundent
            var bestBlockSize = -1;
            for(var blockSize = 2; blockSize<maxCycleLength; blockSize++){
                // comparing blocks of size 2 
                var blockSizeFailed = false;
                for(var i = 1; i<=blockSize; i++){
                    var val1 = this.visitedTargetList[L-i-1];
                    var val2 = this.visitedTargetList[L-i-blockSize-1];
                    var val3 = this.visitedTargetList[L-i-2*blockSize-1];
                    if(val1!=val2||val1!=val3){
                        blockSizeFailed = true;
                        break; // need to change the block size
                    }
                }

                if(!blockSizeFailed){// minimum block size found 
                    blockSizeFound = true;
                    bestBlockSize = blockSize;
                    break;
                }

            }


            if(blockSizeFound){
                print("Ans:")
                // print(this.visitedTargetList);
                // print(this.unvisitedTargetsInfo);
                // print(blockSize);

                var cycleFound = [];
                var cycleInfo = [];
                var L2 = this.unvisitedTargetsInfo.length;
                for(var i = 1; i<=bestBlockSize; i++){
                    cycleFound.unshift(this.visitedTargetList[L-i-1]);
                    cycleInfo.unshift(this.unvisitedTargetsInfo[L2-i]);
                }
                print("cycleFound: "+cycleFound);
                print("cycleInfo: ");
                print(cycleInfo);

                if(L2>=bestBlockSize){
                    this.explorationBoost(cycleFound,cycleInfo); // to modify theta according to seen info
                }else{
                    print("L2 error: L2 = "+L2+"; block="+bestBlockSize+";")
                    print(this.unvisitedTargetsInfo)
                }

                this.visitedTargetList = []; // reset the memory
                this.unvisitedTargetsInfo = [];
            }else{
                //return -1
            }

        }
        
    }


    this.explorationBoost = function(cycleFound,cycleInfo){
        // lets find the worst neglected neighbor during the past cycle
        var maxUncertaintyFound = 0;
        var bestT_i;
        var bestT_j;
        var bestT_k;

        for(var i = 0; i<cycleFound.length; i++){

            var T_i = cycleFound[i];
            if(i==cycleFound.length-1){
                var T_j = cycleFound[0];    
            }else{
                var T_j = cycleFound[i+1];
            }
            
            for(var j = 0; j<cycleInfo[i].length; j++){
                //print("T_i = "+cycleInfo[i][j][0])
                var T_k = cycleInfo[i][j][1];
                var R_k = cycleInfo[i][j][2];
                if(!cycleFound.includes(T_k) && R_k>maxUncertaintyFound){
                    maxUncertaintyFound = R_k;
                    bestT_i = T_i;
                    bestT_j = T_j;
                    bestT_k = T_k;
                }
            }
        }

        if(maxUncertaintyFound==0){
            // nothing to do, no external neighbor
        }else{
            var T_i = bestT_i;
            var T_k = bestT_k;
            var T_j = bestT_j;

            var P_ik = getPathID(T_i,T_k); 
            var P_kj = getPathID(T_k,T_j); 

            print("Agent "+(this.id+1)+"; T_i="+(T_i+1)+"; T_k="+(T_k+1)+"; T_j="+(T_j+1)+"; R_k ="+maxUncertaintyFound.toFixed(2)+".")
            print("cycle: "+cycleFound)
            this.threshold[T_i][T_k] = projectToPositiveAxis(this.threshold[T_i][T_k] - 0.5*boostingCoefficientAlpha);
            this.threshold[T_i][T_j] = projectToPositiveAxis(this.threshold[T_i][T_j] + 0.5*boostingCoefficientAlpha);
            // if(paths[P_ik].distPath()>paths[P_kj].distPath()){                
            //     this.threshold[T_k][T_j] = 0;
            // }else{
            //     this.threshold[T_k][T_i] = 0;
            // } 
        }


    }



}




function addAnAgent(){

    var x = Math.round(50+Math.random()*(width-100));
    var y = Math.round(50+Math.random()*(height-100));
    var r = Number(document.getElementById("sensingRate").value);

    if(targets.length>0){
        var minimumDistanceFound = sqrt(sq(width)+sq(height));
        var closestTargetID = 0;
        for(var i = 0; i<targets.length; i++){
            var distFound = distP2(new Point2(x,y),targets[i].position);
            if(distFound<minimumDistanceFound){
                closestTargetID = i;
                minimumDistanceFound = distFound;
            }
        }
        agents.push(new Agent(targets[closestTargetID].position.x,targets[closestTargetID].position.y,r));
        agents[agents.length-1].residingTarget = [closestTargetID];
        agents[agents.length-1].initialResidingTarget = closestTargetID;
    }else{
        agents.push(new Agent(x,y,r));
    }
    

    var option1 = document.createElement("option");
    option1.text = nf(agents.length);
    document.getElementById("agentSelectDropdown").add(option1);
    document.getElementById("agentSelectDropdown").selectedIndex = agents.length;

}


function addAnAgentAtTarget(targetID){

    var r = Number(document.getElementById("sensingRate").value);
    
    agents.push(new Agent(targets[targetID].position.x,targets[targetID].position.y,r));
    agents[agents.length-1].residingTarget = [targetID];
    agents[agents.length-1].initialResidingTarget = targetID;

    
    var option1 = document.createElement("option");
    option1.text = nf(agents.length);
    document.getElementById("agentSelectDropdown").add(option1);
    document.getElementById("agentSelectDropdown").selectedIndex = agents.length;

}

function removeAnAgent(){

    agents.pop();

    document.getElementById("agentSelectDropdown").remove(agents.length+1);
    document.getElementById("agentSelectDropdown").selectedIndex = agents.length;

}

function evalCostOP3(u_j,v_j,rho_ij,coefs){
    var value = (coefs[0]*sq(u_j) + coefs[1]*sq(v_j) + coefs[2]*u_j*v_j + coefs[3]*u_j + coefs[4]*v_j + coefs[5])/(u_j+v_j+rho_ij); 
    return value;
}









