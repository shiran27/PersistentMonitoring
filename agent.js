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

    this.previousTarget; // boosting


    

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
            var nextTarget = this.findNextTarget(i);


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


                // boosting
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
                this.headingDirectionStep = rotateP2(new Point2(this.maxLinearVelocity*deltaT,0),headingAngle);
                this.position = plusP2(this.position, this.headingDirectionStep);
                this.orientation = headingAngle;
            }
        }else{// going from T_i to T_j (as this.residingTarget = [T_i, T_j]) 
            ////print("travelling i to j");
            this.position = plusP2(this.position, this.headingDirectionStep);
            if(distP2(this.position,targets[this.residingTarget[0]].position)>distP2(targets[this.residingTarget[1]].position,targets[this.residingTarget[0]].position)){
                this.position = targets[this.residingTarget[1]].position;
                this.residingTarget = [this.residingTarget[1]]; 
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
        eventTimeSensitivity = this.savedEventTimeSensitivity;
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
                    }else{
                        eventTimeSensitivity[z][p][q] = -1*(targets[j].sensitivityOfUncertainty[z][p][q])/(targets[j].uncertaintyRate - targets[j].getNetAgentSensingRate());
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
        eventTimeSensitivity = this.savedEventTimeSensitivity;
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
                jArray.push(j);    
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

    this.assignToTheTarget = function(targetID){
        this.position = targets[targetID].position;
        this.residingTarget = [targetID];
        this.initialResidingTarget = targetID;
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









