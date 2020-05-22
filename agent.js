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
    this.coverednessEventTriggered = false;

    this.currentPath;
    this.velocity = 0;
    this.acceleration = 0;
    this.controlProfile = [];
    this.travleTimeSpent = 0;
    this.energySpent = 0; 
    this.energySpentOld = 0;


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


        if(RHCMethod>=8){
            var scalingFac = 2000;
            var barLength = 50; // i.e., 1 000 000 energy
            var numOfBars = Math.floor(this.energySpent/(scalingFac*barLength));
            var remainder = (this.energySpent%(scalingFac*barLength))/(scalingFac);
            
            var barWidth = 3;

            for(var i=1; i<=numOfBars; i++){
                stroke(0,128);
                line(this.position.x,this.position.y+i*this.graphicSizeParameter/2,this.position.x+barLength,this.position.y+i*this.graphicSizeParameter/2);

                rectMode(CORNER);
                fill(255,0,255,150);
                noStroke();
                rect(this.position.x,this.position.y+i*this.graphicSizeParameter/2-barWidth,barLength,2*barWidth);
            
            }
            var i = numOfBars+1;
            stroke(0,128);
            line(this.position.x,this.position.y+i*this.graphicSizeParameter/2,this.position.x+barLength,this.position.y+i*this.graphicSizeParameter/2);

            rectMode(CORNER);
            fill(255,0,255,150);
            noStroke();
            rect(this.position.x,this.position.y+i*this.graphicSizeParameter/2-barWidth,remainder,2*barWidth);
            
            
        }
        
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
                
                ////this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                //// Randomization 2
                if(RHCNoiseEnabled&&RHCNoisev_Max>0){
                    this.headingDirectionStep = rotateP2(new Point2((this.maxLinearVelocity+this.maxLinearVelocity*RHCNoisev_Max*2*(math.random()-0.5)/100)*deltaT,0),headingAngle);    
                }else{
                    this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                }
                //// end Randomization 2

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
            
            //// Randomization 3:
            if(RHCNoiseEnabled && RHCNoiseY_iMagnitude>0){
                this.position = targets[i].position;
            }
            //// end Randomization 3

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
                ////Randomization 2
                ////this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                if(RHCNoiseEnabled&&RHCNoisev_Max>0){
                    this.headingDirectionStep = rotateP2(new Point2((this.maxLinearVelocity+this.maxLinearVelocity*RHCNoisev_Max*2*(math.random()-0.5)/100)*deltaT,0),headingAngle);    
                }else{
                    this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                }
                ////end Randomization 2

                this.position = plusP2(this.position, this.headingDirectionStep);
                this.orientation = headingAngle;
            }
        }else{// going from T_i to T_j (as this.residingTarget = [T_i, T_j]) 
            var i = this.residingTarget[0];// where we were
            var j = this.residingTarget[1];// where we are heading
            var angle = this.orientation;
            ////print("travelling i to j");
            

            ////Randomization 3
            var conditionTemp;
            if(RHCNoiseEnabled && RHCNoiseY_iMagnitude>0){
                var headingAngle = atan2P2(this.position,targets[j].position);
                var rotationRequired = headingAngle-this.orientation;
                for(var k = 0; k<this.graphicBaseShape.length ; k++){
                    this.graphicBaseShapeRotated[k] = rotateP2(this.graphicBaseShapeRotated[k], rotationRequired);
                }
                ////print("need to go to j; rotated");
                this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                this.orientation = headingAngle;
                this.position = plusP2(this.position, this.headingDirectionStep);
                conditionTemp = distP2(this.position,targets[j].position)<1
                //// end Randomization 3
            }else{
                this.position = plusP2(this.position, this.headingDirectionStep);
                conditionTemp = distP2(this.position,targets[i].position)>distP2(targets[j].position,targets[i].position)
            }
            ////this.position = plusP2(this.position, this.headingDirectionStep);
            ////if(distP2(this.position,targets[i].position)>distP2(targets[j].position,targets[i].position)){
            if(conditionTemp){    
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


    // Evet driven optimal RHC update for an agent
    this.updateEDORHCCT = function(){
        // update the agent position s_a(t) of agent a

        if(this.residingTarget.length==1){//residing in some target
            
            var i = this.residingTarget[0];
            var j = i;
            var rho_ij = 0;

            //// Randomization 3:
            if(RHCNoiseEnabled && RHCNoiseY_iMagnitude>0){
                this.position = targets[i].position;
            }
            //// end Randomization 3

            // only at inital soln
            if(this.timeToExitMode[0]==3 && this.timeToExitMode[1]<simulationTime){
                // solve OP-1 to find the u_i
                var u_i = targets[i].uncertainty/(this.sensingRate-targets[i].uncertaintyRate);
                // no need to solve - OP1(i);
                this.timeToExitMode = [1,simulationTime+u_i];

            }else if(this.timeToExitMode[0]==1){// in sensing mode 
                if(this.timeToExitMode[1]<simulationTime){//and sensing time elepased
                    if(targets[i].uncertainty>0){// leave early?
                        // solve OP-3 to find the next target j to visit
                        print("Error-1:early leaving prohibited!")
                    }else{
                        // solve OP-2 to find the sleeping time v_i
                        var v_i;
                        if(RHCMethod==8){
                            v_i = this.solveORHCP1FO(i);
                        }else if(RHCMethod==9){
                            v_i = this.solveORHCP1SO(i);
                        }
                        this.timeToExitMode = [2,simulationTime+v_i];
                        //// if v_i = 0, jump directly to moving: Solving OP-3
                    }
                 
                 }else{
                    // waiting till the end of the predetermined sensing period
                    // this.position = this.position; j = i;
                    if(this.coverednessEventTriggered){
                        ////print("Recompute1 Agent "+this.id);
                        this.coverednessEventTriggered = false;
                        // solve OP-1 to find the u_i 
                        var u_i = targets[i].uncertainty/(this.sensingRate-targets[i].uncertaintyRate);
                        this.timeToExitMode = [1,simulationTime+u_i];
                    
                    }
                 
                 }

            }else if(this.timeToExitMode[0]==2){// in sleeping mode
                if(this.timeToExitMode[1]<simulationTime){//and sleeping time elepased
                    // Solve OP-3 to find the next target to visit
                    ////print("here")
                    var ans;
                    if(RHCMethod==8){
                        ans = this.solveORHCP3FO(i);
                    }else if(RHCMethod==9){
                        ans = this.solveORHCP3SO(i);
                    }

                    j = ans[0];
                    rho_ij = ans[1]; 
                    
                    if(j!=i){
                        this.timeToExitMode = [3,simulationTime+rho_ij];
                        this.controlProfile = [ans[2],ans[3],ans[4],ans[5]]; // [acc,t_1,t_2,E_ij] or [rho_ij,v_1,v_2,E_ij] respectively for first and second order 
                    };
                
                }else{
                    // waiting till the end of the predetermined sleeping period    
                    // this.position = this.position; j = i;
                    if(this.coverednessEventTriggered){
                        ////print("Recompute2 Agent "+this.id);
                        this.coverednessEventTriggered = false;

                        // solve OP-2 to find the sleeping time v_i
                        var v_i;
                        if(RHCMethod==8){
                            v_i = this.solveORHCP1FO(i);
                        }else if(RHCMethod==9){
                            v_i = this.solveORHCP1SO(i);
                        }
                        this.timeToExitMode = [2,simulationTime+v_i];
                    }

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
                
                this.currentPath = getPathID(i,j)
                if(RHCMethod==8){
                    this.acceleration = this.controlProfile[0];
                    if(this.controlProfile[1] < deltaT){print("Max velocity error!")}
                    this.headingDirectionStep = rotateP2(new Point2(0.5*this.acceleration*sq(deltaT),0),headingAngle);
                    this.velocity = this.acceleration*deltaT;
                    //this.energySpent = this.energySpent + sq(this.acceleration)*deltaT;
                    this.energySpentOld = this.energySpent;
                    this.energySpent = this.energySpent + this.controlProfile[3]*deltaT/(2*this.controlProfile[1]);
                }else if(RHCMethod==9){
                    // rho_ij is already loaded: anyway
                    rho_ij = this.controlProfile[0]
                    var v_1 = this.controlProfile[1]
                    var v_2 = this.controlProfile[2]
                    var E_ij = this.controlProfile[3]
                                 
                    this.acceleration = (-1/(2*RHCalpha2))*(v_2+v_1*rho_ij)
                    this.velocity = (-1/(2*RHCalpha2))*((v_2+v_1*rho_ij)*deltaT - 0.5*v_1*sq(deltaT));
                    var distTraveled = (-1/(2*RHCalpha2))*((v_2+v_1*rho_ij)*0.5*sq(deltaT) - v_1*sq(deltaT)*deltaT/6);
                    this.headingDirectionStep = rotateP2(new Point2(distTraveled,0),headingAngle);
                    //this.energySpent = this.energySpent + sq(this.acceleration)*deltaT; // approximation for now
                    this.energySpent = this.energySpent + E_ij*deltaT/rho_ij; 
                    
                }

                this.position = plusP2(this.position, this.headingDirectionStep);
                this.travleTimeSpent = deltaT;
                this.orientation = headingAngle;

                // print("Started to go to j !!! ")
                
                if(dataPlotMode){recordSystemState();}// agent Started Moving towards a new target

                // trigger an event at neighbor targets of i telling i is now uncovered!
                // also trigger an event telling at neighbors of j telling it is now covered!
                this.triggerCoverednessEvent(i,j);
            }
        }else{// going from T_i to T_j (as this.residingTarget = [T_i, T_j]) 
            var i = this.residingTarget[0];// where we were
            var j = this.residingTarget[1];// where we are heading
            var angle = this.orientation;
            ////print("travelling i to j");
            
            if(RHCMethod==8){// FO model
                if(this.travleTimeSpent <= this.controlProfile[1]){
                    // still accelerating
                    this.headingDirectionStep = rotateP2(new Point2(this.velocity*deltaT+0.5*this.acceleration*sq(deltaT),0),angle);       
                    this.velocity = this.velocity + this.acceleration*deltaT;
                    //this.energySpent = this.energySpent + sq(this.acceleration)*deltaT;
                    this.energySpent = this.energySpent + this.controlProfile[3]*deltaT/(2*this.controlProfile[1]);
                }else if(this.travleTimeSpent <= this.controlProfile[2]){
                    this.headingDirectionStep = rotateP2(new Point2(this.velocity*deltaT,0),angle);       
                    this.velocity = this.velocity;
                }else{
                    this.headingDirectionStep = rotateP2(new Point2(this.velocity*deltaT-0.5*this.acceleration*sq(deltaT),0),angle);       
                    this.velocity = this.velocity - this.acceleration*deltaT;
                    //this.energySpent = this.energySpent + sq(this.acceleration)*deltaT;
                    this.energySpent = this.energySpent + this.controlProfile[3]*deltaT/(2*this.controlProfile[1]);
                }
                
                this.position = plusP2(this.position, this.headingDirectionStep);
            
            }else if(RHCMethod==9){
                rho_ij = this.controlProfile[0]
                var v_1 = this.controlProfile[1]
                var v_2 = this.controlProfile[2]
                var E_ij = this.controlProfile[3]

                this.acceleration = (-1/(2*RHCalpha2))*(v_2+v_1*(rho_ij-this.travleTimeSpent));
                var t = this.travleTimeSpent + deltaT
                this.velocity = (-1/(2*RHCalpha2))*((v_2+v_1*rho_ij)*t - 0.5*v_1*sq(t));
                var distTraveled = (-1/(2*RHCalpha2))*((v_2+v_1*rho_ij)*0.5*sq(t) - v_1*sq(t)*t/6);
                this.headingDirectionStep = rotateP2(new Point2(distTraveled,0),angle);
                //this.energySpent = this.energySpent + sq(this.acceleration)*deltaT; // approximation for now
                this.energySpent = this.energySpent + E_ij*deltaT/rho_ij; 
                
                this.position = plusP2(targets[i].position, this.headingDirectionStep);
            }

            
            this.travleTimeSpent = this.travleTimeSpent + deltaT;


            var conditionTemp;
            if(RHCMethod==8){
                conditionTemp = Math.abs(distP2(this.position,targets[i].position)-distP2(targets[j].position,targets[i].position)) < 0.2 //0.2
            }else{
                conditionTemp = Math.abs(distP2(this.position,targets[i].position)-distP2(targets[j].position,targets[i].position)) < 0.01
            }
            
            ////this.position = plusP2(this.position, this.headingDirectionStep);
            ////if(distP2(this.position,targets[i].position)>distP2(targets[j].position,targets[i].position)){
            if(conditionTemp){
                // print("Stopped at j !!! ")
            

                // Final energy corrections
                if(RHCMethod==8){
                    this.energySpent = this.energySpentOld+this.controlProfile[3];
                }else if(RHCMethod==9){
                    this.energySpent = this.energySpent+(this.controlProfile[0]-this.travleTimeSpent)*this.controlProfile[3]/this.controlProfile[0]    
                }

                terminalEnergySpentTemp += this.controlProfile[3]

                // print("Energy:" +this.controlProfile[3])
                // print("Energy:" +this.energySpent)
                
                
                this.acceleration = 0;
                this.velocity = 0;
                this.position = targets[j].position;
                this.travleTimeSpent = 0;
                this.residingTarget = [j]; 
                this.controlProfile = [];
                

                // event driven decision making
                if(this.timeToExitMode[0]==3 && this.timeToExitMode[1]<simulationTime){
                    // solve OP-1 to decide u_i
                    var u_j = targets[j].uncertainty/(this.sensingRate-targets[j].uncertaintyRate);
                    this.timeToExitMode = [1, simulationTime+u_j];
                    //// if u_i = 0 solve OP-3 to find the next target j
                }

                if(dataPlotMode){recordSystemState();}// record agent reached destination target event
            }


        }
    }




    //// Second order OP1
    this.solveORHCP1SO = function(i){
        

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
                    var y_ij = targets[i].distancesToNeighbors[k];
                    yArray.push(y_ij);
                    Abar = Abar + targets[j].uncertaintyRate;    
                    Rbar = Rbar + targets[j].uncertainty;    
                }
                
            }
        }


        var bestDestinationCost = Infinity;
        var bestDestination = i;
        var bestDestinationSolution = [Infinity,0,NaN,NaN];
        var bestDestinationSolutionType;

        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k]; // This is the distance now

            var sol1_1 = this.solveORHCP1SO_1_1(i,j,t_h2,y_ij,Abar,Rbar);
            if(sol1_1[0] < bestDestinationCost){
                bestDestinationCost = sol1_1[0];
                bestDestinationSolution = [...sol1_1];
                bestDestination = j;
                bestDestinationSolutionType = "1_1";
            }

            var sol1_2 = this.solveORHCP1SO_1_2(i,j,t_h2,y_ij,Abar,Rbar);
            if(sol1_2[0] < bestDestinationCost){
                bestDestinationCost = sol1_2[0];
                bestDestinationSolution = [...sol1_2];
                bestDestination = j;
                bestDestinationSolutionType = "1_2";
            }


            var sol1_3 = this.solveORHCP1SO_1_3(i,j,t_h2,y_ij,Abar,Rbar);
            if(sol1_3[0] < bestDestinationCost){
                bestDestinationCost = sol1_3[0];
                bestDestinationSolution = [...sol1_3];
                bestDestination = j;
                bestDestinationSolutionType = "1_3";
            }


            // var sol2_1 = this.solveORHCP1SO_2_1(i,j,t_h2,y_ij,Abar,Rbar);
            // if(sol2_1[0] < bestDestinationCost){
            //     bestDestinationCost = sol2_1[0];
            //     bestDestinationSolution = [...sol2_1];
            //     bestDestination = j;
            //     bestDestinationSolutionType = "2_1";
            // }

            var sol2_2 = this.solveORHCP1SO_2_2(i,j,t_h2,y_ij,Abar,Rbar);
            if(sol2_2[0] < bestDestinationCost){
                bestDestinationCost = sol2_2[0];
                bestDestinationSolution = [...sol2_2];
                bestDestination = j;
                bestDestinationSolutionType = "2_2";
            }


            var sol2_3 = this.solveORHCP1SO_2_3(i,j,t_h2,y_ij,Abar,Rbar);
            if(sol2_3[0] < bestDestinationCost){
                bestDestinationCost = sol2_3[0];
                bestDestinationSolution = [...sol2_3];
                bestDestination = j;
                bestDestinationSolutionType = "2_3";
            }

            var sol3_1 = this.solveORHCP1SO_3_1(i,j,t_h2,y_ij,Abar,Rbar);
            if(sol3_1[0] < bestDestinationCost){
                bestDestinationCost = sol3_1[0];
                bestDestinationSolution = [...sol3_1];
                bestDestination = j;
                bestDestinationSolutionType = "3_1";
            }

            var sol3_2 = this.solveORHCP1SO_3_2(i,j,t_h2,y_ij,Abar,Rbar);
            if(sol3_2[0] < bestDestinationCost){
                bestDestinationCost = sol3_2[0];
                bestDestinationSolution = [...sol3_2];
                bestDestination = j;
                bestDestinationSolutionType = "3_2";
            }



        }

        // print(bestDestinationSolution)
        if(printMode){
            print("Agent "+(this.id+1)+" at Target "+(i+1)+" to Target "+(bestDestination+1))
            print("Best OP2:Case "+bestDestinationSolutionType+"; J= "+bestDestinationSolution[0].toFixed(3)+", v_i= "+bestDestinationSolution[1].toFixed(3)+"; u_j= "+bestDestinationSolution[2].toFixed(3)+", v_j= "+bestDestinationSolution[3].toFixed(3));
        }

        //print(bestDestinationSolution[1])

        return bestDestinationSolution[1];
    
    }


    //// Second order OP1_sub1_1
    this.solveORHCP1SO_1_1 = function(i,j,H,y_ij,Abar,Rbar){

        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        // transit times comes from solving:
        // t_f^4 * Num(DJ) - 36*Den(DJ)*RHCalpha2*y_ij^2==0

        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;
        // var R_i = 0


        // numerator coefficients of: t^3,t^2,t,1
        var sigma_1 = -sq(A_j)*(B_j-A_i) + sq(B_j)*(Abar-A_i)
        var NC1 = sq(A_i*B_j*sigma_1)*B_j
        var NC2 = 3*R_j*NC1/B_j
        var NC3 = A_i*B_j*(3*A_i-B_j)*sq(R_j*sigma_1)
        
        var sigma_2 = A_i*A_j*Math.pow(sigma_1,1.5) // sigma_2*(NC4)^1.5
        var NC4_1 = A_i*sq(B_j)
        var NC4_2 = 2*A_i*B_j*R_j
        var NC4_3 = (A_i-B_j)*sq(R_j)

        var sigma_3 = Math.pow(R_j,3)*A_i
        var NC5 = Math.pow(A_j,4)*sq(A_i)*(A_i-2*B_j) + Math.pow(B_j,3)*(-sq(B_j)*(sq(Abar-A_i-A_j)+sq(A_j))+Math.pow(A_j,3)*(2*B_j-A_j)+A_i*B_j*sq(Abar-A_i-A_j));
        NC5 = NC5 + sq(B_j*A_j)*A_i*( sq(B_j)-4*A_j*B_j+sq(A_j)*(3+2*A_i) ) + (Abar-A_i-A_j)*A_j*sq(B_j)*( A_i*A_j*(-4*B_j+2*A_i)+2*sq(B_j)*(-B_j+A_j+A_i) )
        NC5 = sigma_3*NC5


        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);


        // denominator coefficients: t^2,t,1
        var sigma_4 = B_j*Math.pow(sigma_1,1.5) // denom: sigma_4*(NC4)^1.5
        var sigma_5 = (sigma_2-sigma_4*temp1)



        //// Newton Raphston
        function funVal(t){
            return (NC1*Math.pow(t,3)+NC2*Math.pow(t,2)+NC3*Math.pow(t,1)+NC5) + sigma_5*Math.pow((NC4_1*Math.pow(t,2)+NC4_2*Math.pow(t,1)+NC4_3),1.5)  
        }

        function funDVal(t){
            return (3*NC1*Math.pow(t,2)+2*NC2*Math.pow(t,1)+NC3) + 1.5*sigma_5*Math.sqrt(NC4_1*Math.pow(t,2)+NC4_2*Math.pow(t,1)+NC4_3)*(2*NC4_1*Math.pow(t,1)+NC4_2)  
        }

        var t = 50; // initial guess
        var h = funVal(t) / funDVal(t) 
        var stepCount = 0
        while(Math.abs(h) >= 0.0001){ 
            h = funVal(t) / funDVal(t);  
            t = t - h;
            stepCount += 1
            if(stepCount>1000){
                print('Overflowed 1_1');
                return [Infinity, 0, 0, 0, 0]
            }
        } 
        var rho_ij = t;
        if(rho_ij<0 || rho_ij>H){ return [Infinity, 0, 0, 0, 0]}
        //print("t_f1_1 = "+rho_ij)
        //// End Newton Raphston


        // compute rest: energy and sensing costs
        var v_o2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_o2/rho_ij

        var sigma_5 = Math.sqrt(sigma_1*(NC4_1*sq(rho_ij)+NC4_2*rho_ij+NC4_3)     )
        var sigma_6 = B_j*sigma_1  
        var lambda_i = -rho_ij + (B_j-A_j)*sigma_5/sigma_6 - R_j*(sq(A_j)*(A_i-B_j)+sq(B_j)*(Abar-A_i))/sigma_6
        var tau_j = (R_j+A_j*(lambda_i+rho_ij))/(B_j-A_j)
        var lambda_j = 0        
        if(lambda_i<0 || lambda_j < 0 || (lambda_i+rho_ij+tau_j+lambda_j) > H){ return [Infinity, 0, 0, 0, 0]}

        var t_o = lambda_i;
        var t_f = lambda_i+rho_ij;
        var energyCost = -(Math.pow((-v_o2+v_1*t_o)-v_1*t_f,3)-Math.pow((-v_o2+v_1*t_o)-v_1*t_o,3))/(12*sq(RHCalpha2)*v_1)

        var sensingCost = this.evalSensingCostForORHCP1(i,j,rho_ij,Abar,Rbar,lambda_i,tau_j,lambda_j)
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost
        return [totalCost, lambda_i, rho_ij, tau_j, lambda_j]; 

    }


    //// Second order OP1_sub1_2
    this.solveORHCP1SO_1_2 = function(i,j,H,y_ij,Abar,Rbar){

        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        // transit times comes from solving:
        // t_f^4 * Num(DJ) - 36*Den(DJ)*RHCalpha2*y_ij^2==0

        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;
        // var R_i = 0


        // numerator coefficients of: t^3,t^2,t,1
        var sigma_1 = -sq(A_j)*(B_j-A_i) + sq(B_j)*(Abar-A_i)
        var NC1 = -sq(A_i*B_j*sigma_1)*B_j
        var NC2 = -3*R_j*NC1/B_j
        var NC3 = -A_i*B_j*(3*A_i-B_j)*sq(R_j*sigma_1)
        
        var sigma_2 = A_i*A_j*Math.pow(sigma_1,1.5) // sigma_2*(NC4)^1.5
        var NC4_1 = A_i*sq(B_j)
        var NC4_2 = 2*A_i*B_j*R_j
        var NC4_3 = (A_i-B_j)*sq(R_j)


        var sigma_3 = -Math.pow(R_j,3)*A_i
        var NC5 = Math.pow(A_j,4) * sq(A_i) * (A_i-2*B_j) + Math.pow(B_j,3)*(-sq(B_j)*( sq(Abar-A_i-A_j)+sq(A_j) ) + Math.pow(A_j,3)*(2*B_j-A_j)+A_i*B_j*sq(Abar-A_i-A_j));
        NC5 = NC5 + sq(B_j*A_j)*A_i*( sq(B_j)-4*A_j*B_j+sq(A_j)*(3+2*A_i) ) + (Abar-A_i-A_j)*A_j*sq(B_j)*( A_i*A_j*(-4*B_j+2*A_i)+2*sq(B_j)*(-B_j+A_j+A_i) )
        NC5 = sigma_3*NC5


        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);


        // denominator coefficients: t^2,t,1
        var sigma_4 = B_j*Math.pow(sigma_1,1.5) // denom: sigma_4*(NC4)^1.5
        var sigma_5 = (sigma_2-sigma_4*temp1)



        //// Newton Raphston
        function funVal(t){
            return (NC1*Math.pow(t,3)+NC2*Math.pow(t,2)+NC3*Math.pow(t,1)+NC5) + sigma_5*Math.pow((NC4_1*Math.pow(t,2)+NC4_2*Math.pow(t,1)+NC4_3),1.5)  
        }

        function funDVal(t){
            return (3*NC1*Math.pow(t,2)+2*NC2*Math.pow(t,1)+NC3) + 1.5*sigma_5*Math.sqrt(NC4_1*Math.pow(t,2)+NC4_2*Math.pow(t,1)+NC4_3)*(2*NC4_1*Math.pow(t,1)+NC4_2)  
        }

        var t = 50; // initial guess
        var h = funVal(t) / funDVal(t) 
        var stepCount = 0
        while(Math.abs(h) >= 0.0001){ 
            h = funVal(t) / funDVal(t);  
            t = t - h;
            stepCount += 1
            if(stepCount>1000){
                print('Overflowed 1_2');
                return [Infinity, 0, 0, 0, 0]
            }
        } 
        var rho_ij = t;
        if(rho_ij<0 || rho_ij>H){ return [Infinity, 0, 0, 0, 0]}
        //print("t_f1_2 = "+rho_ij)
        //// End Newton Raphston


        // compute rest: energy and sensing costs
        var v_o2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_o2/rho_ij

        var sigma_5 = Math.sqrt(sigma_1*(NC4_1*sq(rho_ij)+NC4_2*rho_ij+NC4_3)     )
        var sigma_6 = B_j*sigma_1  
        var lambda_i = -rho_ij - (B_j-A_j)*sigma_5/sigma_6 - R_j*(sq(A_j)*(A_i-B_j)+sq(B_j)*(Abar-A_i))/sigma_6
        var tau_j = (R_j+A_j*(lambda_i+rho_ij))/(B_j-A_j)
        var lambda_j = 0        
        if(lambda_i<0 || lambda_j < 0 || (lambda_i+rho_ij+tau_j+lambda_j) > H){ return [Infinity, 0, 0, 0, 0]}

        var t_o = lambda_i;
        var t_f = lambda_i+rho_ij;
        var energyCost = -(Math.pow((-v_o2+v_1*t_o)-v_1*t_f,3)-Math.pow((-v_o2+v_1*t_o)-v_1*t_o,3))/(12*sq(RHCalpha2)*v_1)

        var sensingCost = this.evalSensingCostForORHCP1(i,j,rho_ij,Abar,Rbar,lambda_i,tau_j,lambda_j)
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost
        return [totalCost, lambda_i, rho_ij, tau_j, lambda_j]; ; 

    }



    //// Second order OP1_sub1_3
    this.solveORHCP1SO_1_3 = function(i,j,H,y_ij,Abar,Rbar){

        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        // transit times comes from solving:
        // t_f^4 * Num(DJ) - 36*Den(DJ)*RHCalpha2*y_ij^2==0

        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;
        // var R_i = 0

        // numerator coefficients of: t,1
        var NC1 = A_i*B_j
        var NC2 = A_i*(R_j+A_j*H)
        
        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);

        // denominator coefficients: t^2,t,1
        var DC1 = B_j*H

        var rho_ij = (temp1*DC1 - NC2)/NC1;
        if(rho_ij<0 || rho_ij>H){ return [Infinity, 0, 0, 0, 0]}
        //print("t_f1_3 = "+rho_ij)
        


        // compute rest: energy and sensing costs
        var v_o2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_o2/rho_ij
  
        var lambda_i = -rho_ij + ((B_j-A_j)*H - R_j)/B_j
        var tau_j = (R_j+A_j*(lambda_i+rho_ij))/(B_j-A_j)
        var lambda_j = 0        
        if(lambda_i<0 || lambda_j < 0 || (lambda_i+rho_ij+tau_j+lambda_j) > H){ return [Infinity, 0, 0, 0, 0]}

        var t_o = lambda_i;
        var t_f = lambda_i+rho_ij;
        var energyCost = -(Math.pow((-v_o2+v_1*t_o)-v_1*t_f,3)-Math.pow((-v_o2+v_1*t_o)-v_1*t_o,3))/(12*sq(RHCalpha2)*v_1)

        var sensingCost = this.evalSensingCostForORHCP1(i,j,rho_ij,Abar,Rbar,lambda_i,tau_j,lambda_j)
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost
        return [totalCost, lambda_i, rho_ij, tau_j, lambda_j]; ; 

    }


    //// Second order OP1_sub2_1
    this.solveORHCP1SO_2_1 = function(i,j,H,y_ij,Abar,Rbar){

        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        // transit times comes from solving:
        // t_f^4 * Num(DJ) - 36*Den(DJ)*RHCalpha2*y_ij^2==0

        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;
        // var R_i = 0


        // numerator coefficients of: t^3,t^2,t,1
        var sigma_1 = 2*B_j*(A_j-B_j)*Math.sqrt(Abar-A_j)
        var NC1 = sigma_1*sq(A_j)*B_j
        var NC2 = sigma_1*3*A_j*B_j*R_j
        var NC3 = sigma_1*(A_j+2*B_j)*sq(R_j)
        var NC4 = Math.pow(R_j,3)
                
        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);

        // denominator coefficients: sigma_2*(DC)^1.5
        var sigma_2 = 2*Math.pow(B_j-A_j,1.5) 
        var DC1 = A_j*B_j
        var DC2 = 2*B_j*R_j
        var DC3 = sq(R_j)
        

        //// Newton Raphston
        function funVal(t){
            return (NC1*Math.pow(t,3)+NC2*Math.pow(t,2)+NC3*Math.pow(t,1)+NC4) - temp1*sigma_2*Math.pow((DC1*Math.pow(t,2)+DC2*Math.pow(t,1)+DC3),1.5)  
        }

        function funDVal(t){
           return (3*NC1*Math.pow(t,2)+2*NC2*Math.pow(t,1)+NC3) - 1.5*temp1*sigma_2*Math.sqrt(DC1*Math.pow(t,2)+DC2*Math.pow(t,1)+DC3)*(2*DC1*Math.pow(t,1)+DC2)   
        }

        var t = 50; // initial guess
        var h = funVal(t) / funDVal(t) 
        var stepCount = 0
        while(Math.abs(h) >= 0.0001){ 
            h = funVal(t) / funDVal(t);  
            t = t - h;
            stepCount += 1
            if(stepCount>1000){
                print('Overflowed 2_1');
                return [Infinity, 0, 0, 0, 0]
            }
        }
        var rho_ij = t;
        if(rho_ij<0 || rho_ij>H){ return [Infinity, 0, 0, 0, 0]}
        //print("t_f2_1 = "+rho_ij)
        //// End Newton Raphston


        // compute rest: energy and sensing costs
        var v_o2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_o2/rho_ij

         
        var lambda_i = 0
        var tau_j = (R_j+A_j*(lambda_i+rho_ij))/(B_j-A_j)
        var lambda_j =  -(R_j+B_j*rho_ij)/(B_j-A_j) - Math.sqrt((DC1*sq(rho_ij)+DC2*rho_ij+DC3)/((Abar-A_j)*(B_j-A_j)))       
        if(lambda_i<0 || lambda_j < 0 || (lambda_i+rho_ij+tau_j+lambda_j) > H){ return [Infinity, 0, 0, 0, 0]}
        // this definitely is negative!!!    

        var t_o = lambda_i;
        var t_f = lambda_i+rho_ij;
        var energyCost = -(Math.pow((-v_o2+v_1*t_o)-v_1*t_f,3)-Math.pow((-v_o2+v_1*t_o)-v_1*t_o,3))/(12*sq(RHCalpha2)*v_1)

        var sensingCost = this.evalSensingCostForORHCP1(i,j,rho_ij,Abar,Rbar,lambda_i,tau_j,lambda_j)
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost
        return [totalCost, lambda_i, rho_ij, tau_j, lambda_j]; ; 

    }

    //// Second order OP1_sub2_2
    this.solveORHCP1SO_2_2 = function(i,j,H,y_ij,Abar,Rbar){

        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        // transit times comes from solving:
        // t_f^4 * Num(DJ) - 36*Den(DJ)*RHCalpha2*y_ij^2==0

        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;
        // var R_i = 0


        // numerator coefficients of: t^3,t^2,t,1
        var sigma_1 = -2*B_j*(A_j-B_j)*Math.sqrt(Abar-A_j)
        var NC1 = sigma_1*sq(A_j)*B_j
        var NC2 = sigma_1*3*A_j*B_j*R_j
        var NC3 = sigma_1*(A_j+2*B_j)*sq(R_j)
        var NC4 = Math.pow(R_j,3)
                
        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);

        // denominator coefficients: sigma_2*(DC)^1.5
        var sigma_2 = 2*Math.pow(B_j-A_j,1.5) 
        var DC1 = A_j*B_j
        var DC2 = 2*B_j*R_j
        var DC3 = sq(R_j)
        

        //// Newton Raphston
        function funVal(t){
            return (NC1*Math.pow(t,3)+NC2*Math.pow(t,2)+NC3*Math.pow(t,1)+NC4) - temp1*sigma_2*Math.pow((DC1*Math.pow(t,2)+DC2*Math.pow(t,1)+DC3),1.5)  
        }

        function funDVal(t){
           return (3*NC1*Math.pow(t,2)+2*NC2*Math.pow(t,1)+NC3) - 1.5*temp1*sigma_2*Math.sqrt(DC1*Math.pow(t,2)+DC2*Math.pow(t,1)+DC3)*(2*DC1*Math.pow(t,1)+DC2)   
        }

        var t = 50; // initial guess
        var h = funVal(t) / funDVal(t) 
        var stepCount = 0
        while(Math.abs(h) >= 0.0001){ 
            h = funVal(t) / funDVal(t);  
            t = t - h;
            stepCount += 1
            if(stepCount>1000){
                print('Overflowed 2_2');
                return [Infinity, 0, 0, 0, 0]
            }
        } 
        var rho_ij = t;
        if(rho_ij<0 || rho_ij>H){ return [Infinity, 0, 0, 0, 0]}
        //print("t_f2_2 = "+rho_ij)
        //// End Newton Raphston


        // compute rest: energy and sensing costs
        var v_o2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_o2/rho_ij

         
        var lambda_i = 0
        var tau_j = (R_j+A_j*(lambda_i+rho_ij))/(B_j-A_j)
        var lambda_j =  (R_j+B_j*rho_ij)/(B_j-A_j) + Math.sqrt((DC1*sq(rho_ij)+DC2*rho_ij+DC3)/((Abar-A_j)*(B_j-A_j)))       
        if(lambda_i<0 || lambda_j < 0 || (lambda_i+rho_ij+tau_j+lambda_j) > H){ return [Infinity, 0, 0, 0, 0]}
        // this definitely is positive!!!    

        var t_o = lambda_i;
        var t_f = lambda_i+rho_ij;
        var energyCost = -(Math.pow((-v_o2+v_1*t_o)-v_1*t_f,3)-Math.pow((-v_o2+v_1*t_o)-v_1*t_o,3))/(12*sq(RHCalpha2)*v_1)

        var sensingCost = this.evalSensingCostForORHCP1(i,j,rho_ij,Abar,Rbar,lambda_i,tau_j,lambda_j)
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost
        return [totalCost, lambda_i, rho_ij, tau_j, lambda_j]; ; 

    }


    //// Second order OP1_sub2_3
    this.solveORHCP1SO_2_3 = function(i,j,H,y_ij,Abar,Rbar){

        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        // transit times comes from solving:
        // t_f^4 * Num(DJ) - 36*Den(DJ)*RHCalpha2*y_ij^2==0

        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;
        // var R_i = 0

        // numerator coefficients of: t,1
        var NC1 = -A_j*B_j
        var NC2 = -B_j*R_j
        
        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);

        // denominator coefficients: t^2,t,1
        var DC1 = (A_j-B_j)*H

        var rho_ij = (temp1*DC1 - NC2)/NC1;
        if(rho_ij<0 || rho_ij>H){ return [Infinity, 0, 0, 0, 0]}
        //print("t_f2_3 = "+rho_ij)
        


        // compute rest: energy and sensing costs
        var v_o2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_o2/rho_ij
  
        var lambda_i = 0
        var tau_j = (R_j+A_j*(lambda_i+rho_ij))/(B_j-A_j)
        var lambda_j = H - (R_j+B_j*rho_ij)/(B_j-A_j)        
        if(lambda_i<0 || lambda_j < 0 || (lambda_i+rho_ij+tau_j+lambda_j) > H){ return [Infinity, 0, 0, 0, 0]}

        var t_o = lambda_i;
        var t_f = lambda_i+rho_ij;
        var energyCost = -(Math.pow((-v_o2+v_1*t_o)-v_1*t_f,3)-Math.pow((-v_o2+v_1*t_o)-v_1*t_o,3))/(12*sq(RHCalpha2)*v_1)

        var sensingCost = this.evalSensingCostForORHCP1(i,j,rho_ij,Abar,Rbar,lambda_i,tau_j,lambda_j)
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost
        return [totalCost, lambda_i, rho_ij, tau_j, lambda_j]; ; 

    }

    //// Second order OP1_sub3_1
    this.solveORHCP1SO_3_1 = function(i,j,H,y_ij,Abar,Rbar){

        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        // transit times comes from solving:
        // t_f^4 * Num(DJ) - 36*Den(DJ)*RHCalpha2*y_ij^2==0

        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;
        // var R_i = 0

        // numerator coefficients of: t,1
        var NC1 = A_i*A_j*B_j
        var NC2 = A_i*B_j*(R_j+A_j*H)
        
        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);

        // denominator coefficients: t^2,t,1
        var DC1 = (A_i*B_j-A_i*A_j+A_j*B_j)*H

        var rho_ij = (temp1*DC1 - NC2)/NC1;
        if(rho_ij<0 || rho_ij>H){ return [Infinity, 0, 0, 0, 0]}
        //print("t_f3_1 = "+rho_ij)
        


        // compute rest: energy and sensing costs
        var v_o2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_o2/rho_ij
  
        var lambda_i = -(-A_i*H*(B_j-A_j)+B_j*(R_j+A_j*rho_ij))/(A_i*(B_j-A_j)+A_j*B_j)
        var tau_j = (R_j+A_j*(lambda_i+rho_ij))/(B_j-A_j)
        
        var lambda_j = B_j*(sq(A_j)-A_i*A_j+A_i*B_j)*rho_ij + ( R_j*(A_i*B_j-A_i*A_j-sq(B_j)+2*A_j*B_j) + B_j*A_j*H*(-B_j+A_j) + lambda_i*A_j*(-A_i*A_j+A_j*B_j+A_i*B_j) )
        lambda_j = lambda_j/((A_j-B_j)*(A_i*B_j-A_i*A_j+A_j*B_j))

        if(lambda_i<0 || lambda_j < 0 || (lambda_i+rho_ij+tau_j+lambda_j) > H){ return [Infinity, 0, 0, 0, 0]}

        var t_o = lambda_i;
        var t_f = lambda_i+rho_ij;
        var energyCost = -(Math.pow((-v_o2+v_1*t_o)-v_1*t_f,3)-Math.pow((-v_o2+v_1*t_o)-v_1*t_o,3))/(12*sq(RHCalpha2)*v_1)

        var sensingCost = this.evalSensingCostForORHCP1(i,j,rho_ij,Abar,Rbar,lambda_i,tau_j,lambda_j)
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost
        return [totalCost, lambda_i, rho_ij, tau_j, lambda_j]; ; 

    }


    //// Second order OP1_sub3_2
    this.solveORHCP1SO_3_2 = function(i,j,H,y_ij,Abar,Rbar){

        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        // transit times comes from solving:
        // t_f^4 * Num(DJ) - 36*Den(DJ)*RHCalpha2*y_ij^2==0

        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;
        // var R_i = 0

        // numerator coefficients of: t^2,t,1
        var sigma_1 = -sq(A_j)+B_j*Abar
        var NC1 = -sq(B_j)*sigma_1
        var NC2 = -2*B_j*R_j*sigma_1
        var NC3 = -B_j*sq(R_j)*(Abar-2*A_j+B_j)
        
        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);

        // denominator coefficients: t^2,t,1
        var DC1 = 2*sq(B_j)*(A_j-B_j)
        var DC2 = 4*B_j*R_j*(A_j-B_j)
        var DC3 = 2*sq(R_j)*(A_j-B_j)

        //// Newton Raphston
        function funVal(t){
            return (NC1*Math.pow(t,2)+NC2*Math.pow(t,1)+NC3) - temp1*(DC1*Math.pow(t,2)+DC2*Math.pow(t,1)+DC3)
        }

        function funDVal(t){
           return (2*NC1*Math.pow(t,1)+NC2) - temp1*(2*DC1*Math.pow(t,1)+DC2)
        }

        var t = 10; // initial guess
        var h = funVal(t) / funDVal(t) 
        var stepCount = 0
        while(Math.abs(h) >= 0.0001){ 
            h = funVal(t) / funDVal(t);  
            t = t - h;
            stepCount += 1
            if(stepCount>1000){
                print('Overflowed 3_2');
                // print(funVal(t))
                return [Infinity, 0, 0, 0, 0]
            }
        }
        var rho_ij = t;
        if(rho_ij<0 || rho_ij>H){ return [Infinity, 0, 0, 0, 0]}
        //print("t_f3_2 = "+rho_ij)
        //// End Newton Raphston


        // compute rest: energy and sensing costs
        var v_o2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_o2/rho_ij
  
        var lambda_i = 0
        var tau_j = (R_j+A_j*(lambda_i+rho_ij))/(B_j-A_j)
        var lambda_j = 0
        if(lambda_i<0 || lambda_j < 0 || (lambda_i+rho_ij+tau_j+lambda_j) > H){ return [Infinity, 0, 0, 0, 0]}

        var t_o = lambda_i;
        var t_f = lambda_i+rho_ij;
        var energyCost = -(Math.pow((-v_o2+v_1*t_o)-v_1*t_f,3)-Math.pow((-v_o2+v_1*t_o)-v_1*t_o,3))/(12*sq(RHCalpha2)*v_1)

        var sensingCost = this.evalSensingCostForORHCP1(i,j,rho_ij,Abar,Rbar,lambda_i,tau_j,lambda_j)
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost
        return [totalCost, lambda_i, rho_ij, tau_j, lambda_j]; ; 

    }





    this.evalSensingCostForORHCP1 = function(i,j,y_ij,Abar,Rbar,lambda_i,tau_j,lambda_j){
        // y_ij is now transit time (i.e., rho_ij)
        var coefA = (Abar-targets[i].uncertaintyRate)*sq(lambda_i);
        var coefB = (Abar-this.sensingRate)*sq(tau_j);
        var coefC = (Abar-targets[j].uncertaintyRate)*sq(lambda_j);
        var coefD = 2*(Abar-targets[i].uncertaintyRate)*lambda_i*tau_j;
        var coefE = 2*(Abar-targets[i].uncertaintyRate-targets[j].uncertaintyRate)*lambda_i*lambda_j;
        var coefF = 2*(Abar-targets[j].uncertaintyRate)*tau_j*lambda_j;
        var coefG = 2*((Rbar-targets[i].uncertainty)+(Abar-targets[i].uncertaintyRate)*y_ij)*lambda_i;
        var coefH = 2*((Rbar-targets[i].uncertainty)+Abar*y_ij)*tau_j;
        var coefK = 2*((Rbar-targets[i].uncertainty-targets[j].uncertainty)+(Abar-targets[j].uncertaintyRate)*y_ij)*lambda_j;
        var coefL = y_ij*(2*(Rbar-targets[i].uncertainty)+Abar*y_ij);

        var denJ_s = lambda_i+y_ij+tau_j+lambda_j;
        var numJ_s = coefA + coefB + coefC + coefD + coefE + coefF + coefG + coefH + coefK + coefL;

        return numJ_s/denJ_s;

    }



    //// Second order OP3
    this.solveORHCP3SO = function(i){

        var t_h3 = Math.min(periodT-simulationTime,timeHorizonForRHC);
        
        var jArray = []; // set of candidate targets
        var yArray = []; // distances
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
                    var y_ij = targets[i].distancesToNeighbors[k];
                    yArray.push(y_ij);
                    Abar = Abar + targets[j].uncertaintyRate;    
                    Rbar = Rbar + targets[j].uncertainty;    
                }
                
            }
        }


        var bestDestinationCost = Infinity;
        var bestDestination = i;
        var bestDestinationTime = 0;
        var bestDestinationSolutionType;
        var bestDestinationSolution = [Infinity,NaN,NaN,NaN];

        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k];// now this is distance 
            
            // alpha can be a function of the degree at target i (currently residing)
            var alpha;
            alpha = 1/sq(targets[i].neighbors.length); 
            ////alpha = 0.5;
            //print("Agent "+(this.id+1)+" at Target "+(i+1)+" to Target "+(j+1))

            
            var sol1 = this.solveORHCP3SO_1(i,j,t_h3,y_ij,alpha,Abar,Rbar);
            // [totalCost, energyCost, sensingCost, rho_ij, v_1, v_2, y_ij]
            if(sol1[0]<bestDestinationCost){
                bestDestinationCost = sol1[0];
                bestDestination = j;
                bestDestinationTime = sol1[3];
                bestDestinationSolution = [...sol1];
                bestDestinationSolutionType = 1; 
            }


            var sol2 = this.solveORHCP3SO_2(i,j,t_h3,y_ij,alpha,Abar,Rbar);
            // [totalCost, energyCost, sensingCost, rho_ij, v_1, v_2, y_ij]
            if(sol2[0]<bestDestinationCost){
                bestDestinationCost = sol2[0];
                bestDestination = j;
                bestDestinationTime = sol2[3];
                bestDestinationSolution = [...sol2];
                bestDestinationSolutionType = 2; 
            }


            var sol3 = this.solveORHCP3SO_3(i,j,t_h3,y_ij,alpha,Abar,Rbar);
            // [totalCost, energyCost, sensingCost, rho_ij, v_1, v_2, y_ij]
            if(sol3[0]<bestDestinationCost){
                bestDestinationCost = sol3[0];
                bestDestination = j;
                bestDestinationTime = sol3[3];
                bestDestinationSolution = [...sol3];
                bestDestinationSolutionType = 3; 
            }

            
            
        }

        if(printMode){
            print("Agent "+(this.id+1)+" at Target "+(i+1)+" to Target "+(bestDestination+1))
            print("Best OP3:Case ("+bestDestinationSolutionType+","+bestDestinationSolution[3]+"); J= "+bestDestinationSolution[0].toFixed(3)+"; u_j= "+bestDestinationSolution[1].toFixed(3)+", v_j= "+bestDestinationSolution[2].toFixed(3));
        }

        if(bestDestination!=i){
            if(RHCvmaxObserved<bestDestinationSolution[6]/bestDestinationSolution[3]){
                RHCvmaxObserved = bestDestinationSolution[6]/bestDestinationSolution[3];
            }
            return [bestDestination, bestDestinationTime, bestDestinationSolution[3], bestDestinationSolution[4], bestDestinationSolution[5], bestDestinationSolution[1]];
        }else{
            return [bestDestination, bestDestinationTime,0,0,0,0];
        }
        
    }



    //// Second order OP3_sub1
    this.solveORHCP3SO_1 = function(i,j,H,y_ij,alpha,Abar,Rbar){


        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        // transit times comes from solving:
        // t_f^4 * Num(DJ) - 36*Den(DJ)*RHCalpha2*y_ij^2==0


        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;


        // numerator coefficients of: t^2,t,1
        var sigma_1 = -alpha*A_j*(B_j-A_j) - (1-alpha)*B_j*(Abar-A_j);
        var NC1 = sq(B_j)*sigma_1;
        var NC2 = 2*B_j*R_j*sigma_1;
        var NC3 = -B_j*sq(R_j)*((1-alpha)*(Abar-A_j)+alpha*(B_j-A_j));

        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);

        // denominator coefficients: t^2,t,1
        var DC1 = -temp1*2*sq(B_j)*(A_j-B_j);
        var DC2 = -temp1*4*B_j*R_j*(A_j-B_j);
        var DC3 = -temp1*2*sq(R_j)*(A_j-B_j);

        //// Newton Raphston
        function funVal(t){
            return NC1*Math.pow(t,6)+NC2*Math.pow(t,5)+NC3*Math.pow(t,4)+DC1*Math.pow(t,2)+DC2*Math.pow(t,1)+DC3
        }

        function funDVal(t){
            return 6*NC1*Math.pow(t,5)+5*NC2*Math.pow(t,4)+4*NC3*Math.pow(t,3)+2*DC1*Math.pow(t,1)+DC2
        }

        var t = 50; // initial guess
        var h = funVal(t) / funDVal(t) 
        var stepCount = 0
        while(Math.abs(h) >= 0.00001){ 
            h = funVal(t) / funDVal(t);  
            t = t - h;
            stepCount += 1
            if(stepCount>1000){
                print('Overflowed 1');
                return [Infinity, 0, 0, 0, 0, 0]
            }
        } 
        var rho_ij = t;
        if(rho_ij<0){ return [Infinity, 0, 0, 0, 0, 0]}
        ////print("t_f1 = "+rho_ij)
        //// End Newton Raphston


        //// Nerdamer method
        // var eqnString = '';
        // eqnString += NC1.toExponential(3).toString()+'*x^6 + '
        // eqnString += NC2.toExponential(3).toString()+'*x^5 + '
        // eqnString += NC3.toExponential(3).toString()+'*x^4 + '
        // eqnString += DC1.toExponential(3).toString()+'*x^2 + '
        // eqnString += DC2.toExponential(3).toString()+'*x + '
        // eqnString += DC3.toExponential(3).toString()+' = 0'

        // // print("Equation: ")
        // // print(eqnString)
        // var sol = nerdamer.solveEquations(eqnString,'x');
        // var rho_ij;
        // for(k = 0;k < sol.length; k++){
        //     rho_ij = Number(sol[k].text('decimals'))
        //     if(0<rho_ij && rho_ij<100){
        //         break;
        //     }
        // }
        // print("t_f = "+rho_ij)
        //// End Nerdamer method


        // compute 
        var v_2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_2/rho_ij


        // print("Test: "+(sq(rho_ij)*(3*v_2+2*v_1*rho_ij)+12*RHCalpha2*y_ij))
        // get u based on v_1,v_2,rho_ij
        var energyCost = -(Math.pow(v_2,3)-Math.pow(v_2+v_1*rho_ij,3))/(12*sq(RHCalpha2)*v_1)

        var tau_j = (R_j+rho_ij*A_j)/(B_j-A_j); 
        var lambda_j = 0;
        if(rho_ij+tau_j+lambda_j>H){ return [Infinity, 0, 0, 0, 0, 0]}
        
        var sensingCost = this.evalSensingCostForORHCP3(j,rho_ij,alpha,Abar,Rbar,tau_j,lambda_j)
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost;
        ///print("J_e = "+energyCost.toFixed(2)+"; J_s = "+sensingCost.toFixed(2)+"; J_T = "+totalCost.toFixed(2));

        return [totalCost, energyCost, sensingCost, rho_ij, v_1, v_2, y_ij]

    }


    //// Second order OP3_sub2
    this.solveORHCP3SO_2 = function(i,j,H,y_ij,alpha,Abar,Rbar){
        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;


        // numerator coefficients of: t^3, t^2, t, 1
        var sigma_1 = -Math.sqrt(alpha*(Abar-A_j))*B_j*(A_j-B_j) 
        var NC1 = sq(A_j)*B_j
        var NC2 = 3*A_j*B_j*R_j
        var NC3 = sq(R_j)*(A_j+2*B_j)
        var NC4 = sq(R_j)*R_j

        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);

        // denominator coefficients: (t^2,t,1)^(3/2)
        var sigma_2 = (B_j-A_j)*Math.sqrt((B_j-A_j)/(1-alpha))
        var sigma_3 = -temp1*sigma_2/sigma_1
        var DC1 = A_j*B_j
        var DC2 = 2*B_j*R_j;
        var DC3 = sq(R_j);


        //// Newton Raphston
        function funVal(t){
            return (NC1*Math.pow(t,3)+NC2*Math.pow(t,2)+NC3*Math.pow(t,1)+NC4) +  sigma_3*Math.pow((DC1*Math.pow(t,2)+DC2*Math.pow(t,1)+DC3),1.5)
        }

        function funDVal(t){
            return (3*NC1*Math.pow(t,2)+2*NC2*Math.pow(t,1)+NC3) +  1.5*sigma_3*Math.sqrt(DC1*Math.pow(t,2)+DC2*Math.pow(t,1)+DC3)*(2*DC1*Math.pow(t,1)+DC2) 
        }

        var t = 100; // initial guess
        var h = funVal(t) / funDVal(t) 
        var stepCount = 0
        while(Math.abs(h) >= 0.00001){ 
            h = funVal(t) / funDVal(t);  
            t = t - h;
            stepCount += 1
            if(stepCount>1000){
                print('Overflowed 2');
                return [Infinity, 0, 0, 0, 0, 0]
            }
        }
        var rho_ij = t;
        if(rho_ij<0){ return [Infinity, 0, 0, 0, 0, 0]}
        ////print("t_f2 = "+rho_ij)
        //// End Newton Raphston


        //// Nerdamer method
        // var eqnString = '';
        // eqnString += NC1.toExponential(3).toString()+'*x^3 + '
        // eqnString += NC2.toExponential(3).toString()+'*x^2 + '
        // eqnString += NC3.toExponential(3).toString()+'*x^1 + '
        // eqnString += NC3.toExponential(3).toString()+' + '
        // eqnString += sigma_3.toExponential(3).toString()+' *('
        // eqnString += DC1.toExponential(3).toString()+'*x^2 + '
        // eqnString += DC2.toExponential(3).toString()+'*x + '
        // eqnString += DC3.toExponential(3).toString()+')^1.5 = 0'

        // // print("Equation: ")
        // // print(eqnString)
        // var sol = nerdamer.solveEquations(eqnString,'x');
        // var rho_ij;
        // for(k = 0;k < sol.length; k++){
        //     rho_ij = Number(sol[k].text('decimals'))
        //     if(0<rho_ij && rho_ij<100){
        //         break;
        //     }
        // }
        // print("t_f = "+rho_ij)
        //// End Nerdamer method


        // compute 
        var v_2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_2/rho_ij


        // print("Test: "+(sq(rho_ij)*(3*v_2+2*v_1*rho_ij)+12*RHCalpha2*y_ij))
        // get u based on v_1,v_2,rho_ij
        var energyCost = -(Math.pow(v_2,3)-Math.pow(v_2+v_1*rho_ij,3))/(12*sq(RHCalpha2)*v_1)

        var tau_j = (R_j+rho_ij*A_j)/(B_j-A_j); 
        var sigma_4 = Math.sqrt((2*R_j*(rho_ij+tau_j)+A_j*sq(rho_ij)+sq(tau_j)*(A_j-B_j)+2*A_j*rho_ij*tau_j)*(alpha/((1-alpha)*(Abar-A_j))));
        var lambda_j = sigma_4-tau_j-rho_ij;
        if(lambda_j<0 || rho_ij+tau_j+lambda_j>H){return [Infinity, 0, 0, 0, 0, 0]}

        var sensingCost = this.evalSensingCostForORHCP3(j,rho_ij,alpha,Abar,Rbar,tau_j,lambda_j)
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost;
        print("J_e = "+energyCost.toFixed(2)+"; J_s = "+sensingCost.toFixed(2)+"; J_T = "+totalCost.toFixed(2));

        return [totalCost, energyCost, sensingCost, rho_ij, v_1, v_2,  y_ij]

    }

    //// Second order OP3_sub3
    this.solveORHCP3SO_3 = function(i,j,H,y_ij,alpha,Abar,Rbar){
        // this alpha is from ED_RHC-alpha method's (i.e., not RHCalpha2)

        var A_i = targets[i].uncertaintyRate;
        var A_j = targets[j].uncertaintyRate;
        var B_i = this.sensingRate;
        var B_j = B_i;
        var R_j = targets[j].uncertainty;


        // numerator coefficients of: t, 1
        var NC1 = -A_j*B_j*alpha 
        var NC2 = -B_j*R_j*alpha

        // RHS of eqn:
        var temp1 = 36*RHCalpha2*sq(y_ij);

        // denominator coefficients of: 1
        var DC1 = H*(A_j-B_j)
        

        var rho_ij = (temp1*DC1-NC2)/NC1;
        if(rho_ij<0 || rho_ij>H){ return [Infinity, 0, 0, 0, 0, 0]}
        ////print("t_f3 = "+rho_ij)
        
        // compute 
        var v_2 = 12*RHCalpha2*y_ij/sq(rho_ij)
        var v_1 = -2*v_2/rho_ij


        // print("Test: "+(sq(rho_ij)*(3*v_2+2*v_1*rho_ij)+12*RHCalpha2*y_ij))
        // get u based on v_1,v_2,rho_ij
        var energyCost = -(Math.pow(v_2,3)-Math.pow(v_2+v_1*rho_ij,3))/(12*sq(RHCalpha2)*v_1)

        var tau_j = (R_j+rho_ij*A_j)/(B_j-A_j); 
        var lambda_j = H-rho_ij-tau_j;
        if(lambda_j<0){return [Infinity, 0, 0, 0, 0, 0]}
        
        var sensingCost = this.evalSensingCostForORHCP3(j,rho_ij,alpha,Abar,Rbar,tau_j,lambda_j)
        
        
        var totalCost = 2*sensingCost + RHCalpha2*energyCost;
        print("J_e = "+energyCost.toFixed(2)+"; J_s = "+sensingCost.toFixed(2)+"; J_T = "+totalCost.toFixed(2));

        return [totalCost, energyCost, sensingCost, rho_ij, v_1, v_2,  y_ij]

    }


    this.evalSensingCostForORHCP3 = function(j,y_ij,alpha,Abar,Rbar,tau_j,lambda_j){
        // y_ij is now transit time (i.e., rho_ij)
        var Ajbar = (Abar-targets[j].uncertaintyRate);
        var Rjbar = (Rbar-targets[j].uncertainty);

        var coefA = alpha*(targets[j].uncertaintyRate-this.sensingRate) + Ajbar*(1-alpha);
        var coefB = (1-alpha)*Ajbar;
        var coefC = 2*(1-alpha)*Ajbar;
        var coefD = 2*alpha*(targets[j].uncertainty + targets[j].uncertaintyRate*y_ij) + 2*(1-alpha)*(Rjbar + y_ij*Ajbar);
        var coefE = 2*(1-alpha)*(Rjbar+Ajbar*y_ij);
        var coefF = alpha*y_ij*(2*targets[j].uncertainty + targets[j].uncertaintyRate*y_ij) + (1-alpha)*y_ij*(2*Rjbar+Ajbar*y_ij);
        
        var denJ_s = y_ij+tau_j+lambda_j;
        var numJ_s = coefA*sq(tau_j) + coefB*sq(lambda_j) + coefC*tau_j*lambda_j + coefD*tau_j + coefE*lambda_j + coefF;

        return numJ_s/denJ_s;

    }



    //// First order OP1
    this.solveORHCP1FO = function(i){
        
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
                    var y_ij = targets[i].distancesToNeighbors[k]/RHCvmax;
                    yArray.push(y_ij);
                    Abar = Abar + targets[j].uncertaintyRate;    
                    Rbar = Rbar + targets[j].uncertainty;    
                }
                
            }
        }


        var bestDestinationCost = Infinity;
        var bestDestination = i;
        var bestDestinationSolution = [Infinity,0,NaN,NaN];
        var bestDestinationSolutionType;

        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k];
            var energyCost = 27*sq(RHCvmax)/(2*y_ij);


            // Each coef is multiplied by 2T
            var coefA = (Abar-targets[i].uncertaintyRate);
            var coefB = (Abar-this.sensingRate);
            var coefC = (Abar-targets[j].uncertaintyRate);
            var coefD = 2*(Abar-targets[i].uncertaintyRate);
            var coefE = 2*(Abar-targets[i].uncertaintyRate-targets[j].uncertaintyRate);
            var coefF = 2*(Abar-targets[j].uncertaintyRate);
            var coefG = 2*((Rbar-targets[i].uncertainty)+(Abar-targets[i].uncertaintyRate)*y_ij);
            var coefH = 2*((Rbar-targets[i].uncertainty)+Abar*y_ij);
            var coefK = 2*((Rbar-targets[i].uncertainty-targets[j].uncertainty)+(Abar-targets[j].uncertaintyRate)*y_ij);
            var coefL = y_ij*(2*(Rbar-targets[i].uncertainty)+Abar*y_ij);

            var coefs = [coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefL];


            var sol1 = this.solveOP2C1(i,j,t_h2,y_ij,coefs);
            var totalCost = 2*sol1[0]+RHCalpha2*energyCost;
            if(totalCost < bestDestinationCost){
                bestDestinationCost = totalCost;
                bestDestination = j;
                bestDestinationSolution = [...sol1];
                bestDestinationSolutionType = 1;
            }

            var sol2 = this.solveOP2C2(i,j,t_h2,y_ij,coefs);
            var totalCost = 2*sol2[0]+RHCalpha2*energyCost;
            if(totalCost < bestDestinationCost){
                bestDestinationCost = totalCost;
                bestDestination = j;
                bestDestinationSolution = [...sol2];
                bestDestinationSolutionType = 2;
            }

        }

        // print(bestDestinationSolution)
        if(printMode){
            print("Agent "+(this.id+1)+" at Target "+(i+1)+" to Target "+(bestDestination+1))
            print("Best OP2:Case "+bestDestinationSolutionType+"; J= "+bestDestinationSolution[0].toFixed(3)+", v_i= "+bestDestinationSolution[1].toFixed(3)+"; u_j= "+bestDestinationSolution[2].toFixed(3)+", v_j= "+bestDestinationSolution[3].toFixed(3));
        }

        return bestDestinationSolution[1];
    
    }



    //// First order OP3
    this.solveORHCP3FO = function(i){

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
                    var y_ij = targets[i].distancesToNeighbors[k]/RHCvmax;
                    yArray.push(y_ij);
                    Abar = Abar + targets[j].uncertaintyRate;    
                    Rbar = Rbar + targets[j].uncertainty;    
                }
                
            }
        }


        var bestDestinationCost = Infinity;
        var bestDestinationEnergy;
        var bestDestination = i;
        var bestDestinationTime = 0;
        var bestDestinationSolutionType;
        var bestDestinationSolution = [Infinity,NaN,NaN,NaN];

        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k];
            //var energyCost = 27*Math.pow(this.maxLinearVelocity,3)/(2*(this.maxLinearVelocity*y_ij))// same as
            var energyCost = 27*sq(RHCvmax)/(2*y_ij);

            // revised coeficients 
            // alpha can be a function of the degree at target i (currently residing)
            var alpha;
            alpha = 1/sq(targets[i].neighbors.length); 
            //alpha = 0.5;
            
            var Ajbar = (Abar-targets[j].uncertaintyRate);
            var Rjbar = (Rbar-targets[j].uncertainty);

            var coefA = alpha*(targets[j].uncertaintyRate-this.sensingRate) + Ajbar*(1-alpha);
            var coefB = (1-alpha)*Ajbar;
            var coefC = 2*(1-alpha)*Ajbar;
            var coefD = 2*alpha*(targets[j].uncertainty + targets[j].uncertaintyRate*y_ij) + 2*(1-alpha)*(Rjbar + y_ij*Ajbar)
            var coefE = 2*(1-alpha)*(Rjbar+Ajbar*y_ij);
            var coefF = alpha*y_ij*(2*targets[j].uncertainty + targets[j].uncertaintyRate*y_ij) + (1-alpha)*y_ij*(2*Rjbar+Ajbar*y_ij);
            
            var coefs = [coefA,coefB,coefC,coefD,coefE,coefF]
            var sol1 = this.solveOP3Quick(i,j,t_h3,y_ij,Abar,coefs);
            var totalCost = 2*sol1[0]+RHCalpha2*energyCost;
            // end revise coefs

            if(totalCost<bestDestinationCost){
                bestDestinationCost = totalCost;
                bestDestinationEnergy = energyCost;
                bestDestination = j;
                bestDestinationTime = y_ij;
                bestDestinationSolution = [...sol1];
                bestDestinationSolutionType = 1; 
            }

            
        }

        if(bestDestinationCost<100000 && printMode){
            print("Agent "+(this.id+1)+" at Target "+(i+1)+" to Target "+(bestDestination+1))
            print("Best OP3:Case ("+bestDestinationSolutionType+","+bestDestinationSolution[3]+"); J= "+bestDestinationSolution[0].toFixed(3)+"; u_j= "+bestDestinationSolution[1].toFixed(3)+", v_j= "+bestDestinationSolution[2].toFixed(3));
        }

        if(bestDestination!=i){
            var acc = 9*RHCvmax/(2*bestDestinationTime);
            var t_1 = 3*RHCvmax/(2*acc); // end acceleration
            var t_2 = bestDestinationTime-t_1; // start deceleration
            return [bestDestination, bestDestinationTime, acc, t_1, t_2, bestDestinationEnergy];
        }else{
            return [bestDestination, bestDestinationTime,0,0,0,0];
        }
        

    }

    // Evet driven RHC update for an agent
    this.updateEDRHCCT = function(){
        // update the agent position s_a(t) of agent a

        if(this.residingTarget.length==1){//residing in some target
            
            var i = this.residingTarget[0];
            var j = i;

            //// Randomization 3:
            if(RHCNoiseEnabled && RHCNoiseY_iMagnitude>0){
                this.position = targets[i].position;
            }
            //// end Randomization 3

            // only at inital soln
            if(this.timeToExitMode[0]==3 && this.timeToExitMode[1]<simulationTime){
                // solve OP-1 to find the u_i
                var u_i = this.solveOP1(i);
                this.timeToExitMode = [1,simulationTime+u_i];

            }else if(this.timeToExitMode[0]==1){// in sensing mode 
                 if(this.timeToExitMode[1]<simulationTime){//and sensing time elepased
                    if(targets[i].uncertainty>0){// leave early?
                        // solve OP-3 to find the next target j to visit
                        var ans;
                        if(RHCMethod==6 || RHCMethod==7){
                            ans = this.solveOP3Extended(i);
                        }else{
                            ans = this.solveOP3(i);
                        }
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
                    if(this.coverednessEventTriggered){
                        ////print("Recompute1 Agent "+this.id);
                        this.coverednessEventTriggered = false;

                        // solve OP-1 to find the u_i 
                        var u_i = this.solveOP1(i);
                        this.timeToExitMode = [1,simulationTime+u_i];
                    
                    }
                 
                 }

            }else if(this.timeToExitMode[0]==2){// in sleeping mode
                if(this.timeToExitMode[1]<simulationTime){//and sleeping time elepased
                    // Solve OP-3 to find the next target to visit
                    var ans;
                    if(RHCMethod==6 || RHCMethod==7){
                        ans = this.solveOP3Extended(i);
                    }else{
                        ans = this.solveOP3(i);
                    }
                    j = ans[0];
                    var rho_ij = ans[1]; 
                    if(j!=i){this.timeToExitMode = [3,simulationTime+rho_ij]};
                
                }else{
                    // waiting till the end of the predetermined sleeping period    
                    // this.position = this.position; j = i;
                    if(this.coverednessEventTriggered){
                        ////print("Recompute2 Agent "+this.id);
                        this.coverednessEventTriggered = false;

                        // solve OP-2 to find the sleeping time v_i
                        var v_i = this.solveOP2(i);
                        this.timeToExitMode = [2,simulationTime+v_i];
                    }

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
                
                ////this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                ////Randomization 2:
                if(RHCNoiseEnabled&&RHCNoisev_Max>0){
                    this.headingDirectionStep = rotateP2(new Point2((this.maxLinearVelocity+this.maxLinearVelocity*RHCNoisev_Max*2*(math.random()-0.5)/100)*deltaT,0),headingAngle);    
                }else{
                    this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                }
                //// End Randomization 2

                this.position = plusP2(this.position, this.headingDirectionStep);
                this.orientation = headingAngle;

                if(dataPlotMode){recordSystemState();}// agent Started Moving towards a new target
            
                // trigger an event at neighbor targets of i telling i is now uncovered!
                // also trigger an event telling at neighbors of j telling it is now covered!
                this.triggerCoverednessEvent(i,j);
            }
        }else{// going from T_i to T_j (as this.residingTarget = [T_i, T_j]) 
            var i = this.residingTarget[0];// where we were
            var j = this.residingTarget[1];// where we are heading
            var angle = this.orientation;
            ////print("travelling i to j");
            

            ////Randomization 3
            var conditionTemp;
            if(RHCNoiseEnabled && RHCNoiseY_iMagnitude>0){
                var headingAngle = atan2P2(this.position,targets[j].position);
                var rotationRequired = headingAngle-this.orientation;
                for(var k = 0; k<this.graphicBaseShape.length ; k++){
                    this.graphicBaseShapeRotated[k] = rotateP2(this.graphicBaseShapeRotated[k], rotationRequired);
                }
                ////print("need to go to j; rotated");
                this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                this.orientation = headingAngle;
                this.position = plusP2(this.position, this.headingDirectionStep);
                conditionTemp = distP2(this.position,targets[j].position)<1
                //// end Randomization 3
            }else{
                this.position = plusP2(this.position, this.headingDirectionStep);
                conditionTemp = distP2(this.position,targets[i].position)>distP2(targets[j].position,targets[i].position)
            }
            ////this.position = plusP2(this.position, this.headingDirectionStep);
            ////if(distP2(this.position,targets[i].position)>distP2(targets[j].position,targets[i].position)){
            if(conditionTemp){
                //print("Stopped at j !!! ")
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

        //// Conventional way: sensing till R_i = 0  is reached
        // var t_h = Math.min(periodT-simulationTime,timeHorizonForRHC);

        // var A_i = targets[i].uncertaintyRate;
        // var B_i = this.sensingRate;
        // var R_i = targets[i].uncertainty;
        // var u_i = R_i/(B_i-A_i);

        // return u_i/2;

        var t_h1 = Math.min(periodT-simulationTime,timeHorizonForRHC);
        
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
        var bestDestinationSolution = [Infinity,0,NaN,NaN,NaN];
        var bestDestinationSolutionType;

        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k];

            // Each coef is multiplied by 2T
            var coefA = (Abar - this.sensingRate);
            var coefB = Abar - targets[i].uncertaintyRate;
            var coefC = Abar - this.sensingRate;
            var coefD = Abar - targets[j].uncertaintyRate;
            var coefE = 2*(Abar - targets[i].uncertaintyRate);
            var coefF = 2*(Abar - this.sensingRate);
            var coefG = 2*(Abar - targets[j].uncertaintyRate - this.sensingRate);
            var coefH = 2*(Abar - targets[i].uncertaintyRate);
            var coefK = 2*(Abar - targets[i].uncertaintyRate - targets[j].uncertaintyRate);
            var coefL = 2*(Abar - targets[j].uncertaintyRate);

            var coefM = 2*(Rbar + (Abar-this.sensingRate)*y_ij);
            var coefN = 2*((Rbar-targets[i].uncertainty) + (Abar - targets[i].uncertaintyRate)*y_ij);
            var coefP = 2*(Rbar + Abar*y_ij);
            var coefQ = 2*((Rbar-targets[j].uncertainty) + (Abar - targets[j].uncertaintyRate)*y_ij);
            var coefS = y_ij*(2*Rbar + Abar*y_ij);
            
            var coefs = [coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefL,coefM,coefN,coefP,coefQ,coefS];


            var sol1 = this.solveOP1C1A(i,j,t_h1,y_ij,coefs);
            if(sol1[0]<bestDestinationCost){
                bestDestinationCost = sol1[0];
                bestDestination = j;
                bestDestinationSolution = [...sol1];
                bestDestinationSolutionType = 1; 
            }

            var sol2 = this.solveOP1C1B(i,j,t_h1,y_ij,coefs);
            if(sol2[0]<bestDestinationCost){
                bestDestinationCost = sol2[0];
                bestDestination = j;
                bestDestinationSolution = [...sol2];
                bestDestinationSolutionType = 2; 
            }

            var sol3 = this.solveOP1C2A(i,j,t_h1,y_ij,coefs);
            if(sol3[0]<bestDestinationCost){
                bestDestinationCost = sol3[0];
                bestDestination = j;
                bestDestinationSolution = [...sol3];
                bestDestinationSolutionType = 3; 
            }
            var sol4 = this.solveOP1C2B(i,j,t_h1,y_ij,coefs);
            if(sol4[0]<bestDestinationCost){
                bestDestinationCost = sol4[0];
                bestDestination = j;
                bestDestinationSolution = [...sol4];
                bestDestinationSolutionType = 4; 
            }
            

        }

        // print(bestDestinationSolution)
        if(printMode){
            print("Agent "+(this.id+1)+" at Target "+(i+1)+" to Target "+(bestDestination+1))
            print("Best OP1:Case "+bestDestinationSolutionType+"; J= "+bestDestinationSolution[0].toFixed(3)+"; u_i= "+bestDestinationSolution[1].toFixed(3)+", v_i= "+bestDestinationSolution[2].toFixed(3)+"; u_j= "+bestDestinationSolution[3].toFixed(3)+", v_j= "+bestDestinationSolution[4].toFixed(3));
        }

        return bestDestinationSolution[1];

    }


    this.solveOP1C1A = function(i,j,t_h1,rho_ij,coefs){

        if(t_h1<rho_ij){// not enough time to visit j
            return [Infinity,0];
        }

        var alpha = (targets[j].uncertainty + targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate);
        var beta = targets[j].uncertaintyRate/(this.sensingRate - targets[j].uncertaintyRate);
        var lambda_i0 = targets[i].uncertainty/(this.sensingRate - targets[i].uncertaintyRate);

        // transform coeficients
        var coefA = coefs[0]; 
        var coefB = coefs[2]; 
        var coefC = coefs[5]; 
        var coefD = coefs[10]; 
        var coefE = coefs[12]; 
        var coefF = coefs[14]; 

        var coefG = 1;
        var coefH = 1;
        var coefK = rho_ij;

        var coefP = beta;
        var coefL = alpha;
        
        var coefQ = 1;
        var coefM = t_h1-rho_ij;

        var coefN = lambda_i0;

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        
        var sol;
        if(RHCMethod==5){
            sol = rationalObjective.solveCompleteFixedH(); // solve for u_i,u_j
        }else{
            sol = rationalObjective.solveComplete(); // solve for u_i,u_j
        }

        var costVal = sol[0];
        var u_i = sol[1]; // u_i, v_i = 0, u_j, v_j=0
        var v_i = 0;
        var u_j = sol[2];
        var v_j = 0;

        if(printMode){print("OP1C1A: u_i="+u_i.toFixed(3)+"; v_i="+v_i.toFixed(3)+"; u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; J="+costVal.toFixed(3));}

        if(costVal<0){
            print("E! J (OP1C1A): "+costVal)
            // print(rationalObjective);
        }
        // var costVal = Infinity;
        // var timeVal = 0;
        return [costVal, u_i, v_i, u_j, v_j]; 

    }


    this.solveOP1C1B = function(i,j,t_h1,rho_ij,coefs){

        var alpha = (targets[j].uncertainty + targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate);
        
        if( t_h1 < (rho_ij+alpha)){// not enough time to visit j
            return [Infinity,0];
        }

        var beta = targets[j].uncertaintyRate/(this.sensingRate - targets[j].uncertaintyRate);
        var lambda_i0 = targets[i].uncertainty/(this.sensingRate - targets[i].uncertaintyRate);

        // transform coeficients
        var coefA = coefs[0] + coefs[5]*beta + coefs[2]*sq(beta); 
        var coefB = coefs[3]; 
        var coefC = coefs[6] + coefs[9]*beta; 
        var coefD = coefs[10] + coefs[5]*alpha + coefs[12]*beta + 2*coefs[2]*alpha*beta; 
        var coefE = coefs[13] + coefs[9]*alpha;
        var coefF = coefs[14] + coefs[12]*alpha + coefs[2]*sq(alpha); 

        var coefG = beta + 1;
        var coefH = 1;
        var coefK = alpha + rho_ij;

        var coefP = 0;
        var coefL = Infinity;
        
        var coefQ = 1 + beta;
        var coefM = t_h1-rho_ij-alpha;

        var coefN = lambda_i0;

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        
        var sol;
        if(RHCMethod==5){
            sol = rationalObjective.solveCompleteFixedH(); // solve for u_i,v_j
        }else{
            sol = rationalObjective.solveComplete(); // solve for u_i,v_j
        }

        var costVal = sol[0];
        var u_i = sol[1]; // u_i, v_i = 0, u_j = lambda_j0(u_i,0), v_j
        var v_i = 0;
        var u_j = alpha + beta*(u_i+v_i);
        var v_j = sol[2];

        if(printMode){print("OP1C1B: u_i="+u_i.toFixed(3)+"; v_i="+v_i.toFixed(3)+"; u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; J="+costVal.toFixed(3));}


        if(costVal<0){
            print("E! J (OP1C1B): "+costVal)
            // print(rationalObjective);
        }

        // var costVal = Infinity;
        // var timeVal = 0;
        return [costVal, u_i, v_i, u_j, v_j]; 

    }

    this.solveOP1C2A = function(i,j,t_h1,rho_ij,coefs){

        var lambda_i0 = targets[i].uncertainty/(this.sensingRate - targets[i].uncertaintyRate);

        if(t_h1 < (rho_ij+lambda_i0)){// not enough time to visit j
            return [Infinity,0];
        }

        var alpha = (targets[j].uncertainty + targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate);
        var beta = targets[j].uncertaintyRate/(this.sensingRate - targets[j].uncertaintyRate);
        

        // transform coeficients
        var coefA = coefs[1]; 
        var coefB = coefs[2]; 
        var coefC = coefs[7];
        var coefD = coefs[11] + coefs[4]*lambda_i0;
        var coefE = coefs[12] + coefs[5]*lambda_i0;
        var coefF = coefs[14] + coefs[10]*lambda_i0 + coefs[0]*sq(lambda_i0); 

        var coefG = 1;
        var coefH = 1;
        var coefK = lambda_i0 + rho_ij;

        var coefP = beta;
        var coefL = alpha + beta*lambda_i0;
        
        var coefQ = 1;
        var coefM = t_h1 - rho_ij - lambda_i0;

        var coefN = Infinity;

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        
        var sol;
        if(RHCMethod==5){
            sol = rationalObjective.solveCompleteFixedH(); // solve for v_i,u_j
        }else{
            sol = rationalObjective.solveComplete(); // solve for v_i,u_j
        }

        var costVal = sol[0];
        var u_i = lambda_i0; // u_i=lambda_i0, v_i, u_j, v_j = 0
        var v_i = sol[1];
        var u_j = sol[2]
        var v_j = 0;

        if(printMode){print("OP1C2A: u_i="+u_i.toFixed(3)+"; v_i="+v_i.toFixed(3)+"; u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; J="+costVal.toFixed(3));}

        if(costVal<0){
            print("E! J (OP1C2A): "+costVal)
            // print(rationalObjective);
            // print("OP1C2A: u_i="+u_i.toFixed(3)+"; v_i="+v_i.toFixed(3)+"; r_ij="+rho_ij.toFixed(3)+"; u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; J="+costVal.toFixed(3));
        }
        // var costVal = Infinity;
        // var timeVal = 0;
        return [costVal, u_i, v_i, u_j, v_j];

    }


    this.solveOP1C2B = function(i,j,t_h1,rho_ij,coefs){

        var lambda_i0 = targets[i].uncertainty/(this.sensingRate - targets[i].uncertaintyRate);
        var alpha = (targets[j].uncertainty + targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate);
        var beta = targets[j].uncertaintyRate/(this.sensingRate - targets[j].uncertaintyRate);
        
        if(t_h1 < (rho_ij + alpha + lambda_i0*(1+beta))){// not enough time to visit j
            return [Infinity,0];
        }

        

        // transform coeficients
        var coefA = coefs[1] + coefs[7]*beta + coefs[2]*sq(beta);  
        var coefB = coefs[3]; 
        var coefC = coefs[8] + coefs[9]*beta;
        var coefD = coefs[11] + coefs[4]*lambda_i0 + coefs[12]*beta + (coefs[5]+coefs[7])*beta*lambda_i0 + 2*coefs[2]*sq(beta)*lambda_i0 + coefs[7]*alpha + 2*coefs[2]*alpha*beta;
        var coefE = coefs[13] + coefs[6]*lambda_i0 + coefs[9]*beta*lambda_i0 + coefs[9]*alpha;
        var coefF = coefs[14] + coefs[10]*lambda_i0 + coefs[0]*sq(lambda_i0) + coefs[12]*beta*lambda_i0 + coefs[5]*beta*sq(lambda_i0) + coefs[2]*sq(beta*lambda_i0) + coefs[12]*alpha + coefs[5]*alpha*lambda_i0 + 2*coefs[2]*alpha*beta*lambda_i0 + coefs[2]*sq(alpha); 

        var coefG = 1 + beta;
        var coefH = 1;
        var coefK = alpha + lambda_i0*(1+beta) + rho_ij;

        var coefP = 0;
        var coefL = Infinity;
        
        var coefQ = 1 + beta;
        var coefM = t_h1 - rho_ij - lambda_i0*(1+beta) - alpha;

        var coefN = Infinity;

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        
        var sol;
        if(RHCMethod==5){
            sol = rationalObjective.solveCompleteFixedH(); // solve for v_i,v_j
        }else{
            sol = rationalObjective.solveComplete(); // solve for v_i,v_j
        }


        var costVal = sol[0];
        var u_i = lambda_i0; // u_i=lambda_i0, v_i, u_j=..., v_j 
        var v_i = sol[1];
        var u_j = alpha + beta*(u_i+v_i);
        var v_j = sol[2];

        if(printMode){print("OP1C2B: u_i="+u_i.toFixed(3)+"; v_i="+v_i.toFixed(3)+"; u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; J="+costVal.toFixed(3));}


        if(costVal<0){
            print("E! J (OP1C2B): "+costVal)
            // print(rationalObjective);
            // print("OP1C2B: u_i="+u_i.toFixed(3)+"; v_i="+v_i.toFixed(3)+"; r_ij="+rho_ij.toFixed(3)+"; u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; J="+costVal.toFixed(3));
        }
        // var costVal = Infinity;
        // var timeVal = 0;
        return [costVal, u_i, v_i, u_j, v_j]; 

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
        var bestDestinationSolution = [Infinity,0,NaN,NaN];
        var bestDestinationSolutionType;

        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k];

            // Each coef is multiplied by 2T
            var coefA = (Abar-targets[i].uncertaintyRate);
            var coefB = (Abar-this.sensingRate);
            var coefC = (Abar-targets[j].uncertaintyRate);
            var coefD = 2*(Abar-targets[i].uncertaintyRate);
            var coefE = 2*(Abar-targets[i].uncertaintyRate-targets[j].uncertaintyRate);
            var coefF = 2*(Abar-targets[j].uncertaintyRate);
            var coefG = 2*((Rbar-targets[i].uncertainty)+(Abar-targets[i].uncertaintyRate)*y_ij);
            var coefH = 2*((Rbar-targets[i].uncertainty)+Abar*y_ij);
            var coefK = 2*((Rbar-targets[i].uncertainty-targets[j].uncertainty)+(Abar-targets[j].uncertaintyRate)*y_ij);
            var coefL = y_ij*(2*(Rbar-targets[i].uncertainty)+Abar*y_ij);

            var coefs = [coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefL];


            var sol1 = this.solveOP2C1(i,j,t_h2,y_ij,coefs);
            if(sol1[0]<bestDestinationCost){
                bestDestinationCost = sol1[0];
                bestDestination = j;
                bestDestinationSolution = [...sol1];
                bestDestinationSolutionType = 1;
            }

            var sol2 = this.solveOP2C2(i,j,t_h2,y_ij,coefs);
            if(sol2[0]<bestDestinationCost){
                bestDestinationCost = sol2[0];
                bestDestination = j;
                bestDestinationSolution = [...sol2];
                bestDestinationSolutionType = 2;
            }

        }

        // print(bestDestinationSolution)
        if(printMode){
            print("Agent "+(this.id+1)+" at Target "+(i+1)+" to Target "+(bestDestination+1))
            print("Best OP2:Case "+bestDestinationSolutionType+"; J= "+bestDestinationSolution[0].toFixed(3)+", v_i= "+bestDestinationSolution[1].toFixed(3)+"; u_j= "+bestDestinationSolution[2].toFixed(3)+", v_j= "+bestDestinationSolution[3].toFixed(3));
        }
        return bestDestinationSolution[1];

    }



    this.solveOP2C1 = function(i,j,t_h2,rho_ij,coefs){

        if(t_h2<rho_ij){// not enough time to visit j
            return [Infinity,0];
        }

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

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        
        var sol;
        if(RHCMethod==5){
            sol = rationalObjective.solveCompleteFixedH(); // solve for v_i,u_j
        }else{
            sol = rationalObjective.solveComplete(); // solve for v_i,u_j
        }

        var costVal = sol[0];
        var v_i = sol[1]; // v_i, u_j, v_j=0
        var u_j = sol[2];
        var v_j = 0;

        if(printMode){print("OP2C1: v_i="+v_i.toFixed(3)+"; u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; J="+costVal.toFixed(3));}

        if(costVal<0){
            print("E! J (OP2C1): "+costVal)
            // print(rationalObjective);
            // print("OP2C1: v_i="+v_i.toFixed(3)+"; r_ij="+rho_ij.toFixed(3)+"; u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; J="+costVal.toFixed(3));
        }
        // var costVal = Infinity;
        // var timeVal = 0;
        return [costVal, v_i, u_j, v_j]; ; 

    }


    this.solveOP2C2 = function(i,j,t_h2,rho_ij,coefs){

        var alpha = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate);
        var beta = targets[j].uncertaintyRate/(this.sensingRate-targets[j].uncertaintyRate);

        if(t_h2<rho_ij+alpha){// not enough time to go according to this plan
            return [Infinity,0];
        }


        // transform coeficients
        var coefA = (coefs[1]*sq(beta) + coefs[3]*beta + coefs[0]);  
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

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        
        var sol;
        if(RHCMethod==5){
            sol = rationalObjective.solveCompleteFixedH(); // solve for v_i,v_j
        }else{
            sol = rationalObjective.solveComplete(); // solve for v_i,v_j
        }
        
        var costVal = sol[0];// v_i, u_j=\lambda_{jo}(v_i), v_j
        var v_i = sol[1]; 
        var u_j = alpha + beta*v_i;
        var v_j = sol[2];

        if(printMode){print("OP2C2: v_i="+v_i.toFixed(3)+"; u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; J="+costVal.toFixed(3));}

        if(costVal<0){
            print("E! J (OP2C2): "+costVal)
            // print(rationalObjective);
            // print("OP2C2: v_i="+v_i.toFixed(3)+"; r_ij="+rho_ij.toFixed(3)+"; u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; J="+costVal.toFixed(3));
        }
        // var costVal = Infinity;
        // var timeVal = 0;
        return [costVal, v_i, u_j, v_j]; 

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
        var bestDestinationSolutionType;
        var bestDestinationSolution = [Infinity,NaN,NaN,NaN];

        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k];

            //// Before revision
            // Each coef is multiplied by 2T
            // var coefA = Abar-this.sensingRate;
            // var coefB = (Abar-targets[j].uncertaintyRate);
            // var coefC = 2*(Abar-targets[j].uncertaintyRate);
            // var coefD = 2*(Rbar+Abar*y_ij);
            // var coefE = 2*((Rbar-targets[j].uncertainty)+(Abar-targets[j].uncertaintyRate)*y_ij);
            // var coefF = y_ij*(2*Rbar+Abar*y_ij);
            // var coefs = [coefA,coefB,coefC,coefD,coefE,coefF]

            // var sol1 = this.solveOP3C1(i,j,t_h3,y_ij,Abar,coefs);
            // if(sol1[0]<bestDestinationCost){
            //     bestDestinationCost = sol1[0];
            //     bestDestination = j;
            //     bestDestinationTime = y_ij;
            //     bestDestinationSolution = [...sol1];
            //     bestDestinationSolutionType = 1; 
            // }

            // var sol2 = this.solveOP3C2(i,j,t_h3,y_ij,Abar,coefs);
            // if(sol2[0]<bestDestinationCost){
            //     bestDestinationCost = sol2[0];
            //     bestDestination = j;
            //     bestDestinationTime = y_ij;
            //     bestDestinationSolution = [...sol1];
            //     bestDestinationSolutionType = 2; 
            // }
            //// end-Before revision

            // revised coeficients 
            // alpha can be a function of the degree at target i (currently residing)
            var alpha;
            if(RHCMethod==4 || RHCMethod==7){
                
                if(RHCParameterOverride){
                    alpha = RHCalpha;
                }else{
                    alpha = 1/sq(targets[i].neighbors.length); 
                    //alpha = 1/(15-targets[j].neighbors.length); 
                    //alpha = targets[i].neighbors.length/(10+targets[i].neighbors.length)
                    //alpha = 0; ////alpha = 0.005;   
                }
                
            }else{
                alpha = 0.5;
            }
            var Ajbar = (Abar-targets[j].uncertaintyRate);
            var Rjbar = (Rbar-targets[j].uncertainty);

            var coefA = alpha*(targets[j].uncertaintyRate-this.sensingRate) + Ajbar*(1-alpha);
            var coefB = (1-alpha)*Ajbar;
            var coefC = 2*(1-alpha)*Ajbar;
            var coefD = 2*alpha*(targets[j].uncertainty + targets[j].uncertaintyRate*y_ij) + 2*(1-alpha)*(Rjbar + y_ij*Ajbar)
            var coefE = 2*(1-alpha)*(Rjbar+Ajbar*y_ij);
            var coefF = alpha*y_ij*(2*targets[j].uncertainty + targets[j].uncertaintyRate*y_ij) + (1-alpha)*y_ij*(2*Rjbar+Ajbar*y_ij);
            
            var coefs = [coefA,coefB,coefC,coefD,coefE,coefF]
            var sol1 = this.solveOP3Quick(i,j,t_h3,y_ij,Abar,coefs);
            // end revise coefs

            if(sol1[0]<bestDestinationCost){
                bestDestinationCost = sol1[0];
                bestDestination = j;
                bestDestinationTime = y_ij;
                bestDestinationSolution = [...sol1];
                bestDestinationSolutionType = 1; 
            }

            
        }

        if(bestDestinationCost<100000 && printMode){
            print("Agent "+(this.id+1)+" at Target "+(i+1)+" to Target "+(bestDestination+1))
            print("Best OP3:Case ("+bestDestinationSolutionType+","+bestDestinationSolution[3]+"); J= "+bestDestinationSolution[0].toFixed(3)+"; u_j= "+bestDestinationSolution[1].toFixed(3)+", v_j= "+bestDestinationSolution[2].toFixed(3));
        }

        return [bestDestination, bestDestinationTime];
    
    }


    this.solveOP3Quick = function(i,j,t_h3,rho_ij,Abar,coefs){

        var lambda_j0 = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate); 
        var lambda_j = Math.min(lambda_j0,t_h3-rho_ij);
        var mu_j = t_h3-(rho_ij+lambda_j0);
        if(lambda_j != lambda_j0){mu_j = 0;}// no reason to search for opt v_j

        // transform coeficients
        var coefA = coefs[0]; 
        var coefB = coefs[1]; 
        var coefC = coefs[2]; 
        var coefD = coefs[3]; 
        var coefE = coefs[4]; 
        var coefF = coefs[5]; 

        var coefG = 1;
        var coefH = 1;
        var coefK = rho_ij;

        var coefP = 0;
        var coefL = Infinity;
        
        var coefQ = 0;
        var coefM = mu_j;

        var coefN = lambda_j;

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        // solve for limited feasible space

        var sol;
        if(RHCMethod==5){
            sol = rationalObjective.solveCompleteJointFixedH(); // solve for optimum (v_i,u_j)
            if(Math.abs((sol[1]+sol[2]+rho_ij)-t_h3)>0.001){
                print('Error !- RHCP3-FixedH')
            }

        }else{
            sol = rationalObjective.solveCompleteJoint(); // solve for optimum (v_i,u_j)
        }
     

        var cost = sol[0];
        var u_j  = sol[1];
        var v_j = sol[2];
        return [cost,u_j,v_j];
    }



    this.solveOP3C1 = function(i,j,t_h3,rho_ij,Abar,coefs){// u_j = ?, v_j = 0
        var lambda_j0 = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate); 
        var lambda_j = Math.min(lambda_j0,t_h3-rho_ij);    
        var u_j;
        var v_j = 0;
        var solType;

        if(lambda_j<0){// not enough time to visit j
            //print("Error lambda_j:"+lambda_j);
            return Infinity;
        }

        var u_jSharp = Abar*rho_ij/(this.sensingRate-Abar);

        var cost = 0;
        if(u_jSharp <= lambda_j && Abar < this.sensingRate){
            cost = evalCostOP3(lambda_j,0,rho_ij,coefs); //u_j = lambda_j
            u_j = lambda_j;
            solType = 1;
            //print("testOP3C1: i="+i+", j="+j+", Cost"+cost+"<"+evalCostOP3(0,0,coefs));
        }else{
            cost = evalCostOP3(0,0,rho_ij,coefs); // u_j = 0
            u_j = 0;
            solType = 2;
            //print("testOP3C1: i="+i+", j="+j+", Cost"+cost+"<"+evalCostOP3(lambda_j,0,coefs));
        }
        
        var cost1 = evalCostOP3(lambda_j,0,rho_ij,coefs);
        var cost2 = evalCostOP3(0,0,rho_ij,coefs);     
        ////print("testOP3C1: i="+i+", j="+j+", Cost1:"+cost1.toFixed(3)+", Cost2:"+cost2.toFixed(3)+", Cost:"+cost.toFixed(3)) 

        if(Math.min(cost1,cost2)!=cost){
            print("Error: u_jSharp: "+u_jSharp+", lambda_j: "+lambda_j)
            print("testOP3C1: i="+i+", j="+j+", Cost1:"+cost1.toFixed(3)+", Cost2:"+cost2.toFixed(3)+", Cost:"+cost.toFixed(3)) 
        }

        if(cost<0){
            print("E! J (OP3C1): "+cost)   
        }

        return [cost,u_j,v_j,solType];

    }

    this.solveOP3C2 = function(i,j,t_h3,rho_ij,Abar,coefs){ // u_j = \lambda_j0, v_j = ?
        var lambda_j0 = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate-targets[j].uncertaintyRate); 
        var u_j = lambda_j0;
        var v_j;
        var solType;

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
            v_j = 0;
            solType = 1;
        }else if (v_jSharp >= mu_j){
            cost = evalCostOP3(lambda_j0,mu_j,rho_ij,coefs); // v_j = mu_j
            v_j = mu_j;
            solType = 2;
        }else{
            cost = evalCostOP3(lambda_j0,v_jSharp,rho_ij,coefs); // v_j = v_jSharp
            v_j = v_jSharp;
            solType = 3;
        }


        if(cost<0){
            print("E! J (OP3C2): "+cost)   
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

        return [cost,u_j,v_j,solType];
    }



    this.solveOP3Extended = function(i){

        var H = Math.min(periodT-simulationTime,timeHorizonForRHC);

        var uncoveredNeighborhood = [];

        var jkArray = []; // set of candidate target sequences 
        var y_jkArray = []; // corresponding travel times
        
        var AbarArray = [];  
        var RbarArray = []; 

        for(var jInd = 0; jInd<targets[i].neighbors.length; jInd++){
            
            var j = targets[i].neighbors[jInd];
            if( j != i ){

                var Abar = targets[j].uncertaintyRate;  
                var Rbar = targets[j].uncertainty;  
                var pathsThroughj = 0;

                for(var kInd = 0; kInd<targets[j].neighbors.length; kInd++){
            
                    var k = targets[j].neighbors[kInd];
                    if( k != j ){

                        // need to find out that no agent is already in or en-route to targets j and k
                        var anotherAgentIsComitted = false;
                        for(var a = 0; a<agents.length; a++){
                            if(agents[a].residingTarget.length==1){
                                if(agents[a].residingTarget[0] == j || (k!=i && agents[a].residingTarget[0]==k)){
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

                            pathsThroughj++;
                            jkArray.push([j,k]);
                            var y_ij = targets[i].distancesToNeighbors[jInd]/this.maxLinearVelocity;
                            var y_jk = targets[j].distancesToNeighbors[kInd]/this.maxLinearVelocity;
                            y_jkArray.push([y_ij,y_jk]);

                            Abar = Abar + targets[k].uncertaintyRate;    
                            Rbar = Rbar + targets[k].uncertainty;    

                            if(!uncoveredNeighborhood.includes(j)){uncoveredNeighborhood.push(j)}
                            if(!uncoveredNeighborhood.includes(k)){uncoveredNeighborhood.push(k)}

                        }

                    }
                }

                for(pathInd = 0; pathInd<pathsThroughj; pathInd++){
                    AbarArray.push(Abar);
                    RbarArray.push(Rbar);
                }

            }

        }


        ////var addedLater = [];
        for(var jkInd = 0; jkInd < jkArray.length; jkInd++){
            var j = jkArray[jkInd][0];
            var Abar = AbarArray[jkInd]; // sum over the neighbors of j including j
            var Rbar = RbarArray[jkInd]; // sum over the neighbors of j including j
            ////addedLater.push([]);
            for(var mInd = 0; mInd<uncoveredNeighborhood.length; mInd++){
                var m = uncoveredNeighborhood[mInd];
                if(!targets[j].neighbors.includes(m)){// as neighbors of j have already been considered
                    
                    ////addedLater[addedLater.length-1].push(m);
                    Abar = Abar + targets[m].uncertaintyRate;
                    Rbar = Rbar + targets[m].uncertainty;
    
                }
            }

            AbarArray[jkInd] = Abar;
            RbarArray[jkInd] = Rbar;

        }

        if(printMode){// works!
            // print("i: "+i+", uncovered: "+uncoveredNeighborhood)
            // print("jk Array:")
            // print(jkArray)
            // print("addedLater:")
            // print(addedLater)
        }

        //// quick way : alpha = 0, beta = 0
        // var minCost = Infinity;
        // var minCostTarget = i;
        // var minCostTime = 0;
        // for(var jkInd = 0; jkInd < jkArray.length; jkInd++){
        //     var j = jkArray[jkInd][0];
        //     var k = jkArray[jkInd][1];

        //     var y_ij = y_jkArray[jkInd][0];
        //     var y_jk = y_jkArray[jkInd][1];
            
        //     var R_jkbar = RbarArray[jkInd]-targets[j].uncertainty-targets[k].uncertainty;
        //     var A_jkbar = AbarArray[jkInd]-targets[j].uncertaintyRate-targets[k].uncertaintyRate;

        //     var alpha = (targets[k].uncertainty + targets[k].uncertaintyRate*(y_ij+y_jk))/(this.sensingRate-targets[k].uncertaintyRate);
        //     var beta = targets[k].uncertaintyRate/(this.sensingRate - targets[k].uncertaintyRate);
        //     var lambda_j0 = (targets[j].uncertainty+targets[j].uncertaintyRate*y_ij)/(this.sensingRate - targets[j].uncertaintyRate);

        //     var cost = R_jkbar + 0.5*A_jkbar*(y_jk+y_ij+lambda_j0+alpha+beta*lambda_j0);
        //     if(cost<minCost){
        //         minCost = cost;
        //         minCostTarget = j;
        //         minCostTime = y_ij;
        //     } 
        // }
        // return [minCostTarget,minCostTime]
        //// end quick way



        var bestDestinationCost = Infinity;
        var bestDestination = i;
        var bestDestination2 = i;
        var bestDestinationTime = 0;
        var bestDestinationSolution = [Infinity,NaN,NaN,NaN,NaN];
        var bestDestinationSolutionType;

        for(var jkInd = 0; jkInd < jkArray.length; jkInd++){
            
            var j = jkArray[jkInd][0];
            var k = jkArray[jkInd][1];
            var y_ij = y_jkArray[jkInd][0];
            var y_jk = y_jkArray[jkInd][1];
            var Abar = AbarArray[jkInd]
            var Rbar = RbarArray[jkInd]


            // Each coef is multiplied by 2T
            var coefs;
            if(RHCMethod==7){

                //// alpha beta mode
                var alpha;
                var beta;
                if(RHCParameterOverride){
                    alpha = RHCalpha;//0.1;
                    beta = RHCbeta;//0.2;
                }else{
                    alpha = 1/sq(uncoveredNeighborhood.length);
                    beta = 1/uncoveredNeighborhood.length;
                }
                var B_j = this.sensingRate;
                var B_k = this.sensingRate;
                var A_j = targets[j].uncertaintyRate;
                var A_k = targets[k].uncertaintyRate;
                var R_j = targets[j].uncertainty;
                var R_k = targets[k].uncertainty;
                var A_jk = Abar-A_j-A_k;
                var R_jk = Rbar-R_j-R_k;

                // multiplied by 2
                var coefA = A_k*beta + alpha*(A_j-B_j)-A_jk*(alpha+beta-1);
                var coefB = A_k*beta - A_jk*(alpha+beta-1);
                var coefC = A_j*alpha + beta*(A_k-B_k)-A_jk*(alpha+beta-1);
                var coefD = A_j*alpha - A_jk*(alpha+beta-1);
                var coefE = 2*(A_k*beta - A_jk*(alpha+beta-1));
                var coefF = 2*(A_k*beta + alpha*(A_j-B_j)-A_jk*(alpha+beta-1));
                var coefG = 2*(alpha*(A_j-B_j)-A_jk*(alpha+beta-1));
                var coefH = 2*(A_k*beta - A_jk*(alpha+beta-1));
                var coefK = 2*(-A_jk*(alpha+beta-1));
                var coefL = 2*(A_j*alpha - A_jk*(alpha+beta-1));

                var coefM = 2*(beta*(R_k+A_k*(y_ij+y_jk))+alpha*(R_j+A_j*(y_ij+y_jk)-B_j*y_jk)+(1-alpha-beta)*(R_jk+A_jk*(y_ij+y_jk)));
                var coefN = 2*(beta*(R_k+A_k*(y_ij+y_jk))+(1-alpha-beta)*(R_jk+A_jk*(y_ij+y_jk)));
                var coefP = 2*(beta*(R_k+A_k*(y_ij+y_jk))+alpha*(R_j+A_j*(y_ij+y_jk))+(1-alpha-beta)*(R_jk+A_jk*(y_ij+y_jk)));
                var coefQ = 2*(alpha*(R_j+A_j*(y_ij+y_jk))+(1-alpha-beta)*(R_jk+A_jk*(y_ij+y_jk)));
                var coefS = (y_ij+y_jk)*(alpha*(2*R_j+A_j*(y_ij+y_jk))+beta*(2*R_k+A_k*(y_ij+y_jk))+(1-alpha-beta)*(2*R_jk+A_jk*(y_ij+y_jk)))
                //// end alpha beta mode

                coefs = [coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefL,coefM,coefN,coefP,coefQ,coefS];

            }else{
                
                var coefA = (Abar - this.sensingRate);
                var coefB = Abar - targets[j].uncertaintyRate;
                var coefC = Abar - this.sensingRate;
                var coefD = Abar - targets[k].uncertaintyRate;
                var coefE = 2*(Abar - targets[j].uncertaintyRate);
                var coefF = 2*(Abar - this.sensingRate);
                var coefG = 2*(Abar - targets[k].uncertaintyRate - this.sensingRate);
                var coefH = 2*(Abar - targets[j].uncertaintyRate);
                var coefK = 2*(Abar - targets[j].uncertaintyRate - targets[k].uncertaintyRate);
                var coefL = 2*(Abar - targets[k].uncertaintyRate);

                var coefM = 2*(Rbar - this.sensingRate*y_jk + Abar*(y_ij+y_jk) );
                var coefN = 2*((Rbar-targets[j].uncertainty) + (Abar - targets[j].uncertaintyRate)*(y_ij+y_jk));
                var coefP = 2*(Rbar + Abar*(y_ij+y_jk));
                var coefQ = 2*((Rbar-targets[k].uncertainty) + (Abar - targets[k].uncertaintyRate)*(y_ij+y_jk));
                var coefS = (y_ij+y_jk)*(2*Rbar + Abar*(y_ij+y_jk));
                
                coefs = [coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefL,coefM,coefN,coefP,coefQ,coefS];
            
            }
            

            if(printMode){print("OP3Ext: i="+(i+1)+", j="+(j+1)+", k="+(k+1));}

            var sol1 = this.solveOP3ExtendedC1A(i,j,k,H,y_ij,y_jk,coefs);
            if(sol1[0]<bestDestinationCost){
                bestDestinationCost = sol1[0];
                bestDestination = j;
                bestDestination2 = k;
                bestDestinationTime = y_ij;
                bestDestinationSolution = [...sol1];
                bestDestinationSolutionType = 1; 
            }

            var sol2 = this.solveOP3ExtendedC1B(i,j,k,H,y_ij,y_jk,coefs);
            if(sol2[0]<bestDestinationCost){
                bestDestinationCost = sol2[0];
                bestDestination = j;
                bestDestination2 = k;
                bestDestinationTime = y_ij;
                bestDestinationSolution = [...sol2];
                bestDestinationSolutionType = 2; 
            }

            var sol3 = this.solveOP3ExtendedC2A(i,j,k,H,y_ij,y_jk,coefs);
            if(sol3[0]<bestDestinationCost){
                bestDestinationCost = sol3[0];
                bestDestination = j;
                bestDestination2 = k;
                bestDestinationTime = y_ij;
                bestDestinationSolution = [...sol3];
                bestDestinationSolutionType = 3; 
            }

            var sol4 = this.solveOP3ExtendedC2B(i,j,k,H,y_ij,y_jk,coefs);
            if(sol4[0]<bestDestinationCost){
                bestDestinationCost = sol4[0];
                bestDestination = j;
                bestDestination2 = k;
                bestDestinationTime = y_ij;
                bestDestinationSolution = [...sol4];
                bestDestinationSolutionType = 4; 
            }
            

        }

        // print(bestDestinationSolution)
        if(printMode){
            print("Agent "+(this.id+1)+" at Target "+(i+1)+" to Target "+(bestDestination+1)+" then to target "+(bestDestination2+1)+".")
            print("Best OP3Extended:Case "+bestDestinationSolutionType+"; J= "+bestDestinationSolution[0].toFixed(3)+"; u_j= "+bestDestinationSolution[1].toFixed(3)+", v_j= "+bestDestinationSolution[2].toFixed(3)+"; u_k= "+bestDestinationSolution[3].toFixed(3)+", v_k= "+bestDestinationSolution[4].toFixed(3));
        }

        if(bestDestinationTime==0 && bestDestination==i){
            return this.solveOP3(i);
        }else{
            return [bestDestination,bestDestinationTime];
        }

    }



    this.solveOP3ExtendedC1A = function(i,j,k,H,rho_ij,rho_jk,coefs){

        if(H<(rho_ij+rho_jk)){// not enough time to visit j
            return [Infinity,0];
        }

        var alpha = (targets[k].uncertainty + targets[k].uncertaintyRate*(rho_ij+rho_jk))/(this.sensingRate-targets[k].uncertaintyRate);
        var beta = targets[k].uncertaintyRate/(this.sensingRate - targets[k].uncertaintyRate);
        var lambda_j0 = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate - targets[j].uncertaintyRate);

        // transform coeficients
        var coefA = coefs[0]; 
        var coefB = coefs[2]; 
        var coefC = coefs[5]; 
        var coefD = coefs[10]; 
        var coefE = coefs[12]; 
        var coefF = coefs[14]; 

        var coefG = 1;
        var coefH = 1;
        var coefK = rho_ij+rho_jk;

        var coefP = beta;
        var coefL = alpha;
        
        var coefQ = 1;
        var coefM = H-(rho_ij+rho_jk);

        var coefN = lambda_j0;

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        
        var sol;
        if(RHCMethod==5){// this wont be the case !
            sol = rationalObjective.solveCompleteFixedH(); // solve for u_i,u_j
        }else{
            sol = rationalObjective.solveComplete(); // solve for u_i,u_j
        }

        var costVal = sol[0];
        var u_j = sol[1]; // u_j, v_j = 0, u_k, v_k=0
        var v_j = 0;
        var u_k = sol[2];
        var v_k = 0;

        if(printMode){print("OP3ExC1A: u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; u_k="+u_k.toFixed(3)+"; v_k="+v_k.toFixed(3)+"; J="+costVal.toFixed(3));}

        if(costVal<0){
            print("E! J (OP3ExC1A): "+costVal)
            // print(rationalObjective);
        }
        // var costVal = Infinity;
        // var timeVal = 0;
        return [costVal, u_j, v_j, u_k, v_k]; 

    }


    this.solveOP3ExtendedC1B = function(i,j,k,H,rho_ij,rho_jk,coefs){

        var alpha = (targets[k].uncertainty + targets[k].uncertaintyRate*(rho_ij+rho_jk))/(this.sensingRate-targets[k].uncertaintyRate);
        
        if( H < (rho_ij+rho_jk+alpha)){// not enough time to visit j under this class
            return [Infinity,0];
        }

        var beta = targets[k].uncertaintyRate/(this.sensingRate - targets[k].uncertaintyRate);
        var lambda_j0 = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate - targets[j].uncertaintyRate);

        // transform coeficients
        var coefA = coefs[0] + coefs[5]*beta + coefs[2]*sq(beta); 
        var coefB = coefs[3]; 
        var coefC = coefs[6] + coefs[9]*beta; 
        var coefD = coefs[10] + coefs[5]*alpha + coefs[12]*beta + 2*coefs[2]*alpha*beta; 
        var coefE = coefs[13] + coefs[9]*alpha;
        var coefF = coefs[14] + coefs[12]*alpha + coefs[2]*sq(alpha); 

        var coefG = beta + 1;
        var coefH = 1;
        var coefK = alpha + (rho_ij + rho_jk);

        var coefP = 0;
        var coefL = Infinity;
        
        var coefQ = 1 + beta;
        var coefM = H-(rho_ij+rho_jk+alpha);

        var coefN = lambda_j0;

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        
        var sol;
        if(RHCMethod==5){
            sol = rationalObjective.solveCompleteFixedH(); // solve for u_j,v_k
        }else{
            sol = rationalObjective.solveComplete(); // solve for u_j,v_k
        }

        var costVal = sol[0];
        var u_j = sol[1]; // u_j, v_j = 0, u_k = lambda_k0(u_j,0), v_k
        var v_j = 0;
        var u_k = alpha + beta*(u_j+v_j);
        var v_k = sol[2];

        if(printMode){print("OP3ExC1B: u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; u_k="+u_k.toFixed(3)+"; v_k="+v_k.toFixed(3)+"; J="+costVal.toFixed(3));}


        if(costVal<0){
            print("E! J (OP3ExC1B): "+costVal)
            // print(rationalObjective);
        }

        // var costVal = Infinity;
        // var timeVal = 0;
        return [costVal, u_j, v_j, u_k, v_k]; 

    }

    this.solveOP3ExtendedC2A = function(i,j,k,H,rho_ij,rho_jk,coefs){

        var lambda_j0 = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate - targets[j].uncertaintyRate);

        if(H < (rho_ij+rho_jk+lambda_j0)){// not enough time to visit j
            return [Infinity,0];
        }

        var alpha = (targets[k].uncertainty + targets[k].uncertaintyRate*(rho_ij+rho_jk))/(this.sensingRate-targets[k].uncertaintyRate);
        var beta = targets[k].uncertaintyRate/(this.sensingRate - targets[k].uncertaintyRate);
        

        // transform coeficients
        var coefA = coefs[1]; 
        var coefB = coefs[2]; 
        var coefC = coefs[7];
        var coefD = coefs[11] + coefs[4]*lambda_j0;
        var coefE = coefs[12] + coefs[5]*lambda_j0;
        var coefF = coefs[14] + coefs[10]*lambda_j0 + coefs[0]*sq(lambda_j0); 

        var coefG = 1;
        var coefH = 1;
        var coefK = lambda_j0 + rho_ij +rho_jk;

        var coefP = beta;
        var coefL = alpha + beta*lambda_j0;
        
        var coefQ = 1;
        var coefM = H - rho_ij - rho_jk - lambda_j0;

        var coefN = Infinity;

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        
        var sol;
        if(RHCMethod==5){
            sol = rationalObjective.solveCompleteFixedH(); // solve for v_i,u_j
        }else{
            sol = rationalObjective.solveComplete(); // solve for v_i,u_j
        }

        var costVal = sol[0];
        var u_j = lambda_j0; // u_j=lambda_j0, v_j, u_k, v_k = 0
        var v_j = sol[1];
        var u_k = sol[2];
        var v_k = 0;

        if(printMode){print("OP3ExtC2A: u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; u_k="+u_k.toFixed(3)+"; v_k="+v_k.toFixed(3)+"; J="+costVal.toFixed(3));}

        if(costVal<0){
            print("E! J (OP3ExtC2A): "+costVal)
            // print(rationalObjective);
            // print("OP3ExtC2A: y_ij="+rho_ij.toFixed(3)+"u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; y_jk="+rho_jk.toFixed(3)+"; u_k="+u_k.toFixed(3)+"; v_k="+v_k.toFixed(3)+"; J="+costVal.toFixed(3));
        }
        // var costVal = Infinity;
        // var timeVal = 0;
        return [costVal, u_j, v_j, u_k, v_k];

    }


    this.solveOP3ExtendedC2B = function(i,j,k,H,rho_ij,rho_jk,coefs){

        var lambda_j0 = (targets[j].uncertainty+targets[j].uncertaintyRate*rho_ij)/(this.sensingRate - targets[j].uncertaintyRate);
        var alpha = (targets[k].uncertainty + targets[k].uncertaintyRate*(rho_ij+rho_jk))/(this.sensingRate-targets[k].uncertaintyRate);
        var beta = targets[k].uncertaintyRate/(this.sensingRate - targets[k].uncertaintyRate);
        
        if(H < (rho_ij + rho_jk + alpha + lambda_j0*(1+beta))){// not enough time to visit j
            return [Infinity,0];
        }

        

        // transform coeficients
        var coefA = coefs[1] + coefs[7]*beta + coefs[2]*sq(beta);  
        var coefB = coefs[3]; 
        var coefC = coefs[8] + coefs[9]*beta;
        var coefD = coefs[11] + coefs[4]*lambda_j0 + coefs[12]*beta + (coefs[5]+coefs[7])*beta*lambda_j0 + 2*coefs[2]*sq(beta)*lambda_j0 + coefs[7]*alpha + 2*coefs[2]*alpha*beta;
        var coefE = coefs[13] + coefs[6]*lambda_j0 + coefs[9]*beta*lambda_j0 + coefs[9]*alpha;
        var coefF = coefs[14] + coefs[10]*lambda_j0 + coefs[0]*sq(lambda_j0) + coefs[12]*beta*lambda_j0 + coefs[5]*beta*sq(lambda_j0) + coefs[2]*sq(beta*lambda_j0) + coefs[12]*alpha + coefs[5]*alpha*lambda_j0 + 2*coefs[2]*alpha*beta*lambda_j0 + coefs[2]*sq(alpha); 

        var coefG = 1 + beta;
        var coefH = 1;
        var coefK = alpha + lambda_j0*(1+beta) + rho_ij + rho_jk;

        var coefP = 0;
        var coefL = Infinity;
        
        var coefQ = 1 + beta;
        var coefM = H - rho_ij - rho_jk - lambda_j0*(1+beta) - alpha;

        var coefN = Infinity;

        var rationalObjective = new RationalObj(coefA,coefB,coefC,coefD,coefE,coefF,coefG,coefH,coefK,coefP,coefL,coefQ,coefM,coefN);
        
        var sol;
        if(RHCMethod==5){
            sol = rationalObjective.solveCompleteFixedH(); // solve for v_i,v_j
        }else{
            sol = rationalObjective.solveComplete(); // solve for v_i,v_j
        }


        var costVal = sol[0];
        var u_j = lambda_j0; // u_j=lambda_j0, v_j, u_k=..., v_k 
        var v_j = sol[1];
        var u_k = alpha + beta*(u_j+v_j);
        var v_k = sol[2];

        if(printMode){print("OP3ExtC2B: u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; u_k="+u_k.toFixed(3)+"; v_k="+v_k.toFixed(3)+"; J="+costVal.toFixed(3));}


        if(costVal<0){
            print("E! J (OP3ExtC2B): "+costVal)
            // print(rationalObjective);
            // print("OP3ExtC2A: y_ij="+rho_ij.toFixed(3)+"u_j="+u_j.toFixed(3)+"; v_j="+v_j.toFixed(3)+"; y_jk="+rho_jk.toFixed(3)+"; u_k="+u_k.toFixed(3)+"; v_k="+v_k.toFixed(3)+"; J="+costVal.toFixed(3));
        }
        // var costVal = Infinity;
        // var timeVal = 0;
        return [costVal, u_j, v_j, u_k, v_k]; 

    }



    this.triggerCoverednessEvent = function(i,j){
        // trigger an event at neighbor targets of i telling i is now uncovered!
        // also trigger an event telling at neighbors of j telling it is now covered!
        for(var a = 0; a<agents.length; a++){
            if(agents[a].residingTarget.length==1 && a != this.id){
                // some other agent (i.e., a) resides in target n
                var n = agents[a].residingTarget[0];
                if(targets[i].neighbors.includes(n) || targets[j].neighbors.includes(n)){
                    agents[a].coverednessEventTriggered = true;
                }

            }
            
        }         
        
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









