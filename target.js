function Target(x, y, r) {

    this.position = new Point2(x,y);
    this.initialPosition = new Point2(x,y);
    this.oldPosition = new Point2(x,y);

    this.id = targets.length;

    this.paths = []; 
    for(var i = 0; i<targets.length; i++){
    	paths.push(new Path(targets[i],this));
        this.paths.push(paths[paths.length-1].id);
        targets[i].paths.push(paths[paths.length-1].id);
    }

    this.graphicSizeParameter = 20;
    this.graphicColor = color(50,50,255);

    this.uncertaintyRate = r; 
    this.initialUncertainty = 0.5;
    this.uncertainty = this.initialUncertainty;
    this.residingAgents = []; // contain ID's of agents whom are residing in the target

    this.sensitivityUncertainty = []; //R'_i(t)

    this.meanUncertainty = this.initialUncertainty;

    this.neighbors = [];
    this.distancesToNeighbors = [];
    this.neighbors2 = []; //[[firstNeighbor, secondNeighbor],[firstNeighbor,secondNeighbor]]
    this.distancesToNeighbors2 = [];
    this.neighborhood = [];

    this.arcBoostingInitiated = false;

    this.RHCNoiseR_iEventTimes = []; // time values where random insertion/removal occurs
    this.RHCNoiseR_iEventIndex = 0;

    this.show = function(){

        fill(this.graphicColor);
        noStroke();
        circle(this.position.x,this.position.y,this.graphicSizeParameter);

        fill(255);
        rectMode(CENTER);
        textAlign(CENTER,CENTER);
        text((this.id+1).toString(),this.position.x,this.position.y,this.graphicSizeParameter,this.graphicSizeParameter);
      
        //// P3 presentation
        // var coordinateString = "("+nf(round(this.position.x))+","+nf(round(this.position.y))+")";
        // fill(0);
        // strokeWeight(1);
        // text(coordinateString, this.position.x+40, this.position.y-15);
    
           
    
        // stroke(0,128);
        // line(this.position.x,this.position.y-this.graphicSizeParameter/2,this.position.x,this.position.y-50);
        
        rectMode(CORNER);
        // fill(255,255,0,200);
        fill(255,255,0,255);
        noStroke();
        // rect(this.position.x-10,this.position.y-this.uncertainty-this.graphicSizeParameter/2,20,this.uncertainty);
        rect(this.position.x-15,this.position.y-this.uncertainty-this.graphicSizeParameter/2,30,this.uncertainty);

        //// P3 presentation
        // fill(0,0,255);
        // rectMode(CENTER);
        // textAlign(CENTER,CENTER);
        // text("R_"+(this.id+1)+"="+this.uncertainty.toFixed(3).toString(),this.position.x+45,this.position.y-this.graphicSizeParameter,this.graphicSizeParameter,this.graphicSizeParameter);

        // fill(0,0,255);
        // rectMode(CENTER);
        // textAlign(CENTER,CENTER);
        // text("J_"+(this.id+1)+"="+this.meanUncertainty.toFixed(3).toString(),this.position.x+45,this.position.y-1.7*this.graphicSizeParameter,this.graphicSizeParameter,this.graphicSizeParameter);


    }

    // Randomization 5: random neighbor uncertainty perturbation
    this.uncertaintyRead = function(){
        if(RHCNoiseEnabled&&RHCNoiseR_j>0){
            var R_jNoisy = this.uncertainty + RHCNoiseR_j*2*(Math.random()-0.5);
            if(R_jNoisy<0){
                R_jNoisy = 0;
            }
            return R_jNoisy;
        }else{
            return this.uncertainty;
        }
    }
    // Randomization 5: random neighbor uncertainty perturbation

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
        // update uncertainty values R_i(t) of target i
        
        if(boostingMode==0 || boostingMethod==3 || boostingMethod==4 || boostingMethod==5){ // normal mode
            
            var netAgentSensingRate = this.getNetAgentSensingRate(); 
            var netUncertaintyGrowthRate = this.uncertaintyRate - netAgentSensingRate;
    
            if(this.uncertainty==0 && netUncertaintyGrowthRate<=0){
                this.uncertainty = 0;
            }else{
                this.uncertainty = this.uncertainty + netUncertaintyGrowthRate*deltaT;
                if(this.uncertainty<0){
                    this.uncertainty = 0;
                }
            }
            // since we are running one whole simulation at a time, re do not need the running-mean value
            this.meanUncertainty = this.meanUncertainty + this.uncertainty;
        
        }else if(boostingMethod == 1 && boostingMode==1){ // neighbor boosting is used and is in boosting mode

            var netAgentSensingRate = this.getNetAgentSensingRateNeighborBoosted();
            var netUncertaintyGrowthRate = this.uncertaintyRate - netAgentSensingRate;
    
            if(this.uncertainty==0 && netUncertaintyGrowthRate<=0){
                this.uncertainty = 0;
            }else{
                this.uncertainty = this.uncertainty + netUncertaintyGrowthRate*deltaT;
                if(this.uncertainty<0){
                    this.uncertainty = 0;
                }
            }
            // since we are running one whole simulation at a time, re do not need the running-mean value
            this.meanUncertainty = this.meanUncertainty + this.uncertainty;
        
        }else if(boostingMethod == 2 && boostingMode==1){ // arc boosting is used and is in boosting mode
            
            var answer = this.getNetAgentSensingRateArcBoosted();
         
            var netAgentSensingRate = answer[0];
            var agentCount = answer[1];
            var resAgent = answer[2];

            var netUncertaintyGrowthRate = this.uncertaintyRate - netAgentSensingRate;
            
            if(agentCount==1 && netUncertaintyGrowthRate<=0 && (this.uncertainty==0||this.arcBoostingInitiated)){
                this.arcBoostingInitiated = true;
                this.pushResidingAgentsAway(resAgent); 
                this.uncertainty = 0; //this.uncertainty - netUncertaintyGrowthRate*deltaT;
            }else{
                this.arcBoostingInitiated = false;
                this.uncertainty = this.uncertainty + netUncertaintyGrowthRate*deltaT;
                if(this.uncertainty<0){
                    this.uncertainty = 0;
                }
                
            }
            this.meanUncertainty = this.meanUncertainty + this.uncertainty;
            // since we are running one whole simulation at a time, re do not need the running-mean value
            
        
        }
        


    }

    



    this.updateFastCT = function(){

        //// Randomization 3:
        if(RHCNoiseEnabled && RHCNoiseY_iMagnitude>0){
            ////first order
            ////this.position.randomPerturbP2(RHCNoiseY_iMagnitude);
            
            ////second order
            var temp = minusP2(productP2(this.position,2),this.oldPosition)
            temp.randomPerturbP2(sq(deltaT)*RHCNoiseY_iMagnitude);
            this.oldPosition = this.position;
            this.position = temp;

            this.position = avoidEscapeP2(boundWithinRadius(this.position,this.initialPosition,RHCNoiseY_iBoundary));
            updateNeighborDistances();
        }
        //// end Randomization 3

        // update uncertainty values R_i(t) of target i
        ////var netUncertaintyGrowthRate = this.uncertaintyRate - this.getNetAgentSensingRate();
        //// Randomization 1:
        if(RHCNoiseEnabled && RHCNoiseA_i>0){
            var netUncertaintyGrowthRate = this.uncertaintyRate + this.uncertaintyRate*RHCNoiseA_i*2*(Math.random()-0.5)/100 - this.getNetAgentSensingRate();
        }else{
            var netUncertaintyGrowthRate = this.uncertaintyRate - this.getNetAgentSensingRate();
        }
        //// end Randomization 1

        if(this.uncertainty==0 && netUncertaintyGrowthRate<=0){
            this.uncertainty = 0;
        }else{
            this.uncertainty = this.uncertainty + netUncertaintyGrowthRate*deltaT;
            if(this.uncertainty<0){
                this.uncertainty = 0;

                if(dataPlotMode){recordSystemState();}// target uncertainty reached zero event
            }
        }
        
        // following line is inaccurate, yet it does not matter as target.meanuncertainty is irrelevent
        // in the middle of the simulation (at the end of the simulation run, we have to divide it)
        this.meanUncertainty = this.meanUncertainty + this.uncertainty;

    }


    this.updateCT = function(){
        // update uncertainty values R_i(t) of target i
        
        ////Randomization 3
        //this.position = avoidEscapeP2(plusP2(this.initialPosition,new Point2(5*(Math.random()-0.5),5*(Math.random()-0.5))));
        if(RHCNoiseEnabled && RHCNoiseY_iMagnitude>0){
            ////first order
            ////this.position.randomPerturbP2(RHCNoiseY_iMagnitude);
            
            ////second order
            var temp = minusP2(productP2(this.position,2),this.oldPosition)
            temp.randomPerturbP2(sq(deltaT)*RHCNoiseY_iMagnitude);
            this.oldPosition = this.position;
            this.position = temp;

            this.position = avoidEscapeP2(boundWithinRadius(this.position,this.initialPosition,RHCNoiseY_iBoundary));
            updateNeighborDistances();
        }
        ////end Randomization 3


        ////var netUncertaintyGrowthRate = this.uncertaintyRate - this.getNetAgentSensingRate();
        //// Randomization 1:
        if(RHCNoiseEnabled && RHCNoiseA_i>0){
            var netUncertaintyGrowthRate = this.uncertaintyRate + this.uncertaintyRate*RHCNoiseA_i*2*(Math.random()-0.5)/100 - this.getNetAgentSensingRate();
        }else{
            var netUncertaintyGrowthRate = this.uncertaintyRate - this.getNetAgentSensingRate();
        }
        //// end Randomization 1

        var oldUncertainty = this.uncertainty;
        if(this.uncertainty==0 && netUncertaintyGrowthRate<=0){
            this.uncertainty = 0;
        }else{
            this.uncertainty = this.uncertainty + netUncertaintyGrowthRate*deltaT;
            if(this.uncertainty<0){
                this.uncertainty = 0;
            }
        }
        this.meanUncertainty = (1/(discreteTimeSteps+1))*(discreteTimeSteps*this.meanUncertainty + (oldUncertainty+this.uncertainty)/2);

    }


    this.updateRHCNoiseR_i = function(){
        var nextEventIndex = this.RHCNoiseR_iEventIndex;
        if(this.RHCNoiseR_iEventTimes.length > nextEventIndex){
            var nextEventTIme = this.RHCNoiseR_iEventTimes[nextEventIndex];
            if(simulationTime>=nextEventTIme){
                this.RHCNoiseR_iEventIndex = nextEventIndex + 1;
                this.uncertainty = this.uncertainty + RHCNoiseR_iMagnitude*2*(Math.random()-0.5);
                if(this.uncertainty<0){
                    this.uncertainty = 0;
                }

                for(var a=0; a<agents.length; a++){
                    if(agents[a].residingTarget.length==1){
                        if(this.neighbors.includes(agents[a].residingTarget[0])){
                            agents[a].coverednessEventTriggered = true;
                        }
                    }
                }

                recordSystemState(); 
            }    
        }
        
    }


    this.updateCT2 = function(){ // not used any more
        // update uncertainty values R_i(t) of target i
        var netUncertaintyGrowthRate = this.uncertaintyRate - this.getNetAgentSensingRate();
        var oldUncertainty = this.uncertainty;
        if(this.uncertainty==0 && netUncertaintyGrowthRate<=0){
            this.uncertainty = 0;
        }else{
            this.uncertainty = this.uncertainty + netUncertaintyGrowthRate*deltaT;
            if(this.uncertainty<0){
                this.uncertainty = 0;
            }
        }
        
        this.meanUncertainty = this.meanUncertainty + this.uncertainty;
    }



    this.getNetAgentSensingRate = function(){//B_i*N_i value
        var sumValue = 0;
        for(var i = 0; i<agents.length; i++){
            if(agents[i].residingTarget.length == 1 && agents[i].residingTarget[0]==this.id){
                sumValue = sumValue + agents[i].sensingRate;
            }
        }
        return sumValue;
    }

    this.getNetAgentSensingRateNeighborBoosted = function(){

        var sumValue = 0;
        var agentCount = 0;
        var factorial = 1; // 0!
        var alpha = boostingCoefficientAlpha;

        for(var i = 0; i<agents.length; i++){
            if(agents[i].residingTarget.length == 1 && agents[i].residingTarget[0]==this.id){
                
                if(agentCount==0){// to avoid numerical computational errors
                    sumValue = sumValue + agents[i].sensingRate*1;// agent will feel the target uncertainty behaves 
                }else{// agentCount = (N_i-1)
                    sumValue = sumValue + agents[i].sensingRate*Math.pow(-1*boostingCoefficientAlpha*agentCount,agentCount)/factorial;    
                }
                
                agentCount = agentCount + 1;
                factorial = factorial*agentCount;
            }
        }

        return sumValue;        
        
        
    }

    
    this.getNetAgentSensingRateArcBoosted = function(){

        var sumValue = 0;
        var agentCount = 0;
        var factorial = 1; // 0!
        var alpha = boostingCoefficientAlpha;
        var resAgent;
        for(var i = 0; i<agents.length; i++){
            if(agents[i].residingTarget.length == 1 && agents[i].residingTarget[0]==this.id){
                
                if(agentCount==0){// to avoid numerical computational errors
                    sumValue = sumValue + agents[i].sensingRate*1;// agent will feel the target uncertainty behaves 
                }else{// agentCount = (N_i-1)
                    sumValue = sumValue + agents[i].sensingRate*Math.pow(-1*boostingCoefficientAlpha*agentCount,agentCount)/factorial;    
                }
                
                agentCount = agentCount + 1;
                resAgent = i;
                factorial = factorial*agentCount;
            }
        }


        // boosting part 2
        return [sumValue, agentCount, resAgent];
        
    }


    this.pushResidingAgentsAway = function(a){
        
        var q = agents[a].findProbableNextTarget(this.id);
        var p = this.id;
        agents[a].threshold[p][q] = 0;      
        
    }

}


function addATarget(){

    var x = Math.round((50+Math.random()*(width-100))/10)*10;
    var y = Math.round((50+Math.random()*(height-100))/10)*10;
    var r = Number(document.getElementById("uncertaintyRate").value);
    targets.push(new Target(x,y,r));


    var option1 = document.createElement("option");
    option1.text = nf(targets.length);
    document.getElementById("targetSelectDropdown").add(option1);
    document.getElementById("targetSelectDropdown").selectedIndex = targets.length;


}

function addATargetAt(x,y){

    var r = Number(document.getElementById("uncertaintyRate").value);
    targets.push(new Target(x,y,r));


    var option1 = document.createElement("option");
    option1.text = nf(targets.length);
    document.getElementById("targetSelectDropdown").add(option1);
    document.getElementById("targetSelectDropdown").selectedIndex = targets.length;


}

function removeATarget(){

    //print(paths);
    var pathIDsToDelete = targets[targets.length-1].paths;
    var pathIDsToDeleteSorted = pathIDsToDelete.sort(function(a, b){return b - a}); //descending sort
    for(var i = 0; i < pathIDsToDeleteSorted.length; i++){

        var indexOfPathToDelete = targets[paths[pathIDsToDeleteSorted[i]].targets[0]].paths.indexOf(pathIDsToDeleteSorted[i]);
        ////print("Rem: "+indexOfPathToDelete)
        targets[paths[pathIDsToDeleteSorted[i]].targets[0]].paths.splice(indexOfPathToDelete,1); // we always remove the last target
        paths.splice(pathIDsToDeleteSorted[i],1);
        // path id pathIDsToDeleteSorted[i] should be also removed from target of that path in the other end
    }
    targets.pop();

    document.getElementById("targetSelectDropdown").remove(targets.length+1);
    document.getElementById("targetSelectDropdown").selectedIndex = targets.length;
    //print(paths);

}

function isNeighbors(id1,id2){
    var idArray = [id1,id2];
    var sortedIDArray = idArray.sort(function(a, b){return a - b});
    for(var j=0; j<targets[sortedIDArray[0]].paths.length; j++){
        if(sortedIDArray[1]==paths[targets[sortedIDArray[0]].paths[j]].targets[1] && paths[targets[sortedIDArray[0]].paths[j]].isPermenent){
            return true
        }
    } 
    if(id1==id2){
        return true;
    }else{
        return false;
    }
}

// function setModeOfAgents(agents,modes){
//     for(var i=0;i<particleShadows.lenghth;i++){
//         particleShadows[i].isBoostingActivated = modes[i];
//     }
// }

function updateNeighborDistances(){
    for(var i = 0; i<targets.length; i++){

        targets[i].distancesToNeighbors = [];
        for(var jInd = 0; jInd<targets[i].neighbors.length; jInd++){
            var j = targets[i].neighbors[jInd];
            if(i==j){
                targets[i].distancesToNeighbors.push(0);
            }else{
                var distij = distP2(targets[i].position,targets[j].position)
                targets[i].distancesToNeighbors.push(distij);
            }
        }

    }
}

