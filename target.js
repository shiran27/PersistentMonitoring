function Target(x, y, r) {

    this.position = new Point2(x,y);
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

    
    this.show = function(){

        fill(this.graphicColor);
        noStroke();
        circle(this.position.x,this.position.y,this.graphicSizeParameter);

        fill(255);
        rectMode(CENTER);
        textAlign(CENTER,CENTER);
        text((this.id+1).toString(),this.position.x,this.position.y,this.graphicSizeParameter,this.graphicSizeParameter);
      
        
        var coordinateString = "("+nf(round(this.position.x))+","+nf(round(this.position.y))+")";
        fill(0);
        strokeWeight(1);
        text(coordinateString, this.position.x+40, this.position.y-15);
    
           
    
        stroke(0,128);
        line(this.position.x,this.position.y-this.graphicSizeParameter/2,this.position.x,this.position.y-50);
        
        rectMode(CORNER);
        fill(255,255,0,150);
        noStroke();
        rect(this.position.x-10,this.position.y-this.uncertainty-this.graphicSizeParameter/2,20,this.uncertainty);

        fill(0,0,255);
        rectMode(CENTER);
        textAlign(CENTER,CENTER);
        text("R_"+(this.id+1)+"="+this.uncertainty.toFixed(3).toString(),this.position.x+45,this.position.y-this.graphicSizeParameter,this.graphicSizeParameter,this.graphicSizeParameter);

        fill(0,0,255);
        rectMode(CENTER);
        textAlign(CENTER,CENTER);
        text("J_"+(this.id+1)+"="+this.meanUncertainty.toFixed(3).toString(),this.position.x+45,this.position.y-1.7*this.graphicSizeParameter,this.graphicSizeParameter,this.graphicSizeParameter);

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
        // update uncertainty values R_i(t) of target i
        var netUncertaintyGrowthRate = this.uncertaintyRate - this.getNetAgentSensingRate();
        var oldUncertainty = this.uncertainty;
        if(this.uncertainty==0 && netUncertaintyGrowthRate<0){
            this.uncertainty = 0;
        }else{
            this.uncertainty = this.uncertainty + netUncertaintyGrowthRate*deltaT;
            if(this.uncertainty<0){
                this.uncertainty = 0;
            }
        }
        
        this.meanUncertainty = this.meanUncertainty + this.uncertainty;


    }

    



    this.updateFastCT = function(){
        // update uncertainty values R_i(t) of target i
        var netUncertaintyGrowthRate = this.uncertaintyRate - this.getNetAgentSensingRate();
        
        if(this.uncertainty==0 && netUncertaintyGrowthRate<0){
            this.uncertainty = 0;
        }else{
            this.uncertainty = this.uncertainty + netUncertaintyGrowthRate*deltaT;
            if(this.uncertainty<0){
                this.uncertainty = 0;
            }
        }
        
        this.meanUncertainty = this.meanUncertainty + this.uncertainty;


    }

    this.updateCT = function(){
        // update uncertainty values R_i(t) of target i
        var netUncertaintyGrowthRate = this.uncertaintyRate - this.getNetAgentSensingRate();
        var oldUncertainty = this.uncertainty;
        if(this.uncertainty==0 && netUncertaintyGrowthRate<0){
            this.uncertainty = 0;
        }else{
            this.uncertainty = this.uncertainty + netUncertaintyGrowthRate*deltaT;
            if(this.uncertainty<0){
                this.uncertainty = 0;
            }
        }
        this.meanUncertainty = (1/(discreteTimeSteps+1))*(discreteTimeSteps*this.meanUncertainty + (oldUncertainty+this.uncertainty)/2);

    }


    this.updateCT2 = function(){
        // update uncertainty values R_i(t) of target i
        var netUncertaintyGrowthRate = this.uncertaintyRate - this.getNetAgentSensingRate();
        var oldUncertainty = this.uncertainty;
        if(this.uncertainty==0 && netUncertaintyGrowthRate<0){
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

