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



    this.updateOneStepGreedyCT = function(){
        // update the agent position s_a(t) of agent a
        if(this.residingTarget.length==1){//residing in some target
            var i = this.residingTarget[0];
            // The following two lines are the only difference compared to "this.updateCT" defined above
            var j = this.findNextTargetOneStepGreedy(i);// gives T_j according tho the threshold polcy
            if(targets[i].uncertainty > 0 || j==i){
                
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



    this.findNextTargetOneStepGreedy = function(currentTargetIndex){

        var i = currentTargetIndex;
        var jArray = []; // set of candidate targets
        var yArray = []; // travel times

        for(var j = 0; j<targets.length; j++){
            if( j != i ){
                //print(paths[getPathID(i,j)].isPermanent)
                var pathIDForij = getPathID(i,j); 
                if(paths[pathIDForij].isPermenent && targets[j].residingAgents.length==0){ 
                    
                    // need to find out that no agent is en-route to target j
                    var anotherAgentIsComitted = false;
                    for(var a = 0; a<agents.length; a++){
                        if(agents[a].residingTarget[1]==j){
                            anotherAgentIsComitted = true;
                            break;
                        }
                    }

                    
                    if(!anotherAgentIsComitted){
                        jArray.push(j);
                        var y_ij = paths[pathIDForij].distPath()/this.maxLinearVelocity;
                        yArray.push(y_ij);    
                    }
                                  
                }
                
            }
        }


        if(jArray.length==0 || targets[i].uncertainty>0){// no need to go to a neighbor
            return i;
        }else if(targets[i].uncertainty==0){
            print(jArray);
            //print(yArray);
        }

        
        // now we need to execute the one step (ahead) greedy selection.
        ////var maxGain = 0;
        var minGain = Infinity;
        var minGainDestinationTarget = i;

        for(var k = 0; k < jArray.length; k++){
            
            var j = jArray[k];
            var y_ij = yArray[k];
            
            var A_i = targets[i].uncertaintyRate;
            var A_j = targets[j].uncertaintyRate;
            var B_j = this.sensingRate; 
            var R_j0 = targets[j].uncertainty;

            var tau_jHat = (R_j0+A_j*y_ij)/(B_j-A_j);
            var tempCond = 2*(B_j-A_j)*(A_i*y_ij)/(B_j-A_i)-A_j*y_ij;

            if(y_ij>(periodT-simulationTime)|| R_j0<tempCond){
                // no point in going to this destination
            }else if((periodT-simulationTime)<tau_jHat){
                var tau_j = (periodT-simulationTime);
                var gainVal;

                if(oneStepAheadGreedyMethod==1){
                    gainVal = A_i*sq(y_ij) + 2*tau_j*y_ij*A_i - (B_j-A_i)*sq(tau_j); //theoretically, this needs to be divided by (2*periodT);
                    ////gainVal = (tau_j*(B_j-A_i)-y_ij*A_i)/periodT;
                }else{
                    gainVal = (A_i*sq(y_ij) + 2*tau_j*y_ij*A_i-(B_j-A_i)*sq(tau_j))/(y_ij+tau_j);
                    ////gainVal = (tau_j*(B_j-A_i)-y_ij*A_i)/(tau_j*periodT)
                }

                if(gainVal<minGain){
                    minGain = gainVal;
                    minGainDestinationTarget = j;
                }
                // jkjkvbsvbsdvbioas
            }else{
                var tau_j = tau_jHat;
                var delta_j = (periodT-simulationTime)-(y_ij+tau_j);
                
                var gainVal;
                if(oneStepAheadGreedyMethod==1){
                    gainVal = A_i*sq(y_ij) + 2*tau_jHat*y_ij*A_i - 2*delta_j*((A_j-A_i)*y_ij+R_j0) - 2*delta_j*tau_jHat*(A_j-A_i) - (B_j-A_i)*sq(tau_jHat) - (A_j-A_i)*sq(delta_j); //theoretically, this needs to be divided by (2*periodT);
                    ////gainVal = (tau_j*(B_j-A_i)+delta_j*(A_j-A_i)-y_ij*A_i)/periodT;
                }else{
                    gainVal = (A_i*sq(y_ij) + 2*tau_jHat*y_ij*A_i - 2*delta_j*((A_j-A_i)*y_ij+R_j0) - 2*delta_j*tau_jHat*(A_j-A_i) - (B_j-A_i)*sq(tau_jHat) - (A_j-A_i)*sq(delta_j))/(y_ij+tau_jHat+delta_j);
                    ////gainVal = (tau_j*(B_j-A_i)+delta_j*(A_j-A_i)-y_ij*A_i)/((tau_j+delta_j)*periodT);
                }
                
                if(gainVal<minGain){
                    minGain = gainVal;
                    minGainDestinationTarget = j;
                }
            }
        }

        return minGainDestinationTarget;

    }

    // first attempt:
    // this.findNextTargetOneStepGreedy = function(currentTargetIndex){

    //     var i = currentTargetIndex;
    //     var jArray = []; // set of candidate targets
    //     var yArray = []; // travel times

    //     for(var j = 0; j<targets.length; j++){
    //         if( j != i ){
    //             //print(paths[getPathID(i,j)].isPermanent)
    //             var pathIDForij = getPathID(i,j); 
    //             if(paths[pathIDForij].isPermenent && targets[j].residingAgents.length==0){ 
                    
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
    //                     var y_ij = paths[pathIDForij].distPath()/this.maxLinearVelocity;
    //                     yArray.push(y_ij);    
    //                 }
                                  
    //             }
                
    //         }
    //     }

    //     if(jArray.length==0 || targets[i].uncertainty>0){// no need to go to a neighbor
    //         return i;
    //     }else if(targets[i].uncertainty==0){
    //         print(jArray);
    //         //print(yArray);
    //     }

        
    //     // now we need to execute the one step (ahead) greedy selection.
    //     var maxGain = 0;
    //     var maxGainDestinationTarget = i;

    //     for(var k = 0; k < jArray.length; k++){
            
    //         var j = jArray[k];
    //         var y_ij = yArray[k];
            
    //         var A_i = targets[i].uncertaintyRate;
    //         var A_j = targets[j].uncertaintyRate;
    //         var B_j = this.sensingRate; 
    //         var R_j0 = targets[j].uncertainty;

    //         var tau_jHat = (R_j0+A_j*y_ij)/(B_j-A_j);

    //         if(y_ij>(periodT-simulationTime)){
    //             // no point in going to this destination
    //         }else if((periodT-simulationTime)<tau_jHat){
    //             var tau_j = (periodT-simulationTime);
                
    //             var gainVal;
    //             if(oneStepAheadGreedyMethod==1){
    //                 gainVal = (tau_j*(B_j-A_i)-y_ij*A_i)/periodT;
    //             }else{
    //                 gainVal = (tau_j*(B_j-A_i)-y_ij*A_i)/(tau_j*periodT)
    //             }

    //             if(gainVal>maxGain){
    //                 maxGain = gainVal;
    //                 maxGainDestinationTarget = j;
    //             }
    //             // jkjkvbsvbsdvbioas
    //         }else{
    //             var tau_j = tau_jHat;
    //             var delta_j = (periodT-simulationTime)-(y_ij+tau_j);
                
    //             var gainVal;
    //             if(oneStepAheadGreedyMethod==1){
    //                 gainVal = (tau_j*(B_j-A_i)+delta_j*(A_j-A_i)-y_ij*A_i)/periodT;
    //             }else{
    //                 gainVal = (tau_j*(B_j-A_i)+delta_j*(A_j-A_i)-y_ij*A_i)/((tau_j+delta_j)*periodT);
    //             }
                
    //             if(gainVal>maxGain){
    //                 maxGain = gainVal;
    //                 maxGainDestinationTarget = j;
    //             }
    //         }
    //     }

    //     return maxGainDestinationTarget;

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









