function Path(target1,target2){

	this.targets = [target1.id,target2.id];

	this.position = new Point2(0.5*(target1.position.x+target2.position.x),0.5*(target1.position.y+target2.position.y));

	this.isPermenent = true;

	this.brokenDueToClustering = false;

	this.inTheCycle = false;

	this.id = paths.length;

	this.graphicSizeParameter = 10;


	this.show = function(){

		//fill(125);
		if(problemConfigurationEditMode){
			
			var target1  = targets[this.targets[0]];
			var target2  = targets[this.targets[1]];
			this.position = new Point2(0.5*(target1.position.x+target2.position.x),0.5*(target1.position.y+target2.position.y));
			

			fill(0);
			noStroke();
        	rectMode(CENTER);
        	textAlign(CENTER,CENTER);
        	text("P_"+(this.id+1),this.position.x+15,this.position.y);


			noStroke()
			fill(255,0,0);
			circle(this.position.x,this.position.y,this.graphicSizeParameter);
			
			if(distP2(this.position,new Point2(mouseX,mouseY))<this.graphicSizeParameter||!this.isPermenent){
				stroke(0,28);
			}else{
				stroke(0);
			}

			

		}else if(this.inTheCycle){
			stroke(200,0,0);
		}else if(this.brokenDueToClustering && this.isPermenent){
			stroke(0,32);
		}else if(this.isPermenent){
			stroke(0);
		}else{
			stroke(0,0);
		}
		line(targets[this.targets[0]].position.x, targets[this.targets[0]].position.y, targets[this.targets[1]].position.x, targets[this.targets[1]].position.y);


	}


	this.highlight = function(colorValue){// to print cycles
		stroke(colorValue);
		strokeWeight(10);
		fill(colorValue);
		line(targets[this.targets[0]].position.x, targets[this.targets[0]].position.y, targets[this.targets[1]].position.x, targets[this.targets[1]].position.y);
		strokeWeight(1);
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

    this.distPath = function(){
    	var T_i = this.targets[0];
    	var T_j = this.targets[1];
    	if(!this.isPermenent){
    		return 10000;
    	}else{
    		return distP2(targets[T_i].position,targets[T_j].position);
    	}
    }



}

function disconnectAllPaths(){
	for(var i = 0; i<paths.length; i++){
		paths[i].isPermenent = false;
	}
}

function connectPathsBetween(targetIDList){
	for(var i = 0; i<targetIDList.length; i=i+2){
		for(var p = 0; p<paths.length; p++){
			if( paths[p].targets[0]==targetIDList[i] && paths[p].targets[1]==targetIDList[i+1] ){
				paths[p].isPermenent = true;
			}
		}
	}
}


function getPathID(targetID1,targetID2){
	for(var i = 0; i<paths.length; i++){
		if(paths[i].targets.includes(targetID1) && paths[i].targets.includes(targetID2)){
			return i;
		}
	}
	print("Error - path finding, between T_"+targetID1+","+targetID2);
}





