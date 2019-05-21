function Path(target1,target2){

	this.targets = [target1.id,target2.id];

	this.position = new Point2(0.5*(target1.position.x+target2.position.x),0.5*(target1.position.y+target2.position.y));

	this.isPermenent = true;

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
		}else if(this.isPermenent){
			stroke(0);
		}else{
			stroke(0,0);
		}
		line(targets[this.targets[0]].position.x, targets[this.targets[0]].position.y, targets[this.targets[1]].position.x, targets[this.targets[1]].position.y);


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



// greedy construction of optimal cycle to travel.- for one agent

function getInitialLoop(){
	var agentID	= 0;
	var targetSet = math.range(0,targets.length)._data;

	var route = getBestInitialPath(agentID,targetSet);
	var newRoute = [...route];
	while(newRoute.length>0){
		print("Current route length:"+route[0].length);
		var newRoute = getBestTargetToAdd(agentID,targetSet,route[0],route[1],route[2]);
		if(newRoute.length>0){
			route = [...newRoute];
		}else{
			////consolePrint("Increase PeriodT !!!");
			
		}
	}
	consolePrint("Steady state mean uncertainty (J) ="+route[2].toFixed(3)+", achieved via greedy loop search.");

	//2 - OPT
	print("2-OPT Begin")
	for(var i = 0; i<route[0].length-2; i++){
		
		for(var k = i+2; k<route[0].length; k++){
			route = swap2OPT(route,i,k,agentID);	
		}
		
	}

	// 3 - Opt
	print("3-OPT Begin")
	for(var i = 0; i<route[0].length-2; i++){
		for(var j = i+1; j<route[0].length-1; j++){
			for(var k = j+1; k<route[0].length; k++){
				route = swap3OPT(route,i,j,k,agentID);
			}	
		}
	}



	//print(route);
	resetInitialLoop();
	for(var p = 0; p<route[0].length; p++){
		paths[route[0][p]].inTheCycle = true;
	}
	print("Result Loop: "+route[1]+", cost: "+route[2].toFixed(2));

	cyclicRoutes[agentID] = route;
	//return route; 

}

function resetInitialLoop(){
	for(var p = 0; p<paths.length; p++){
		paths[p].inTheCycle	= false;
	}
}



function getBestInitialPath(agentID,targetSet){// to initialize the  problem

	var maxGain = -1;
	var bestPath = -1;

	// path means we are adding two targets to the monitoring list
	for(var p = 0; p<paths.length; p++){

		var T_i = paths[p].targets[0];
		var T_j = paths[p].targets[1];
		var A_a = agentID;

		if(targetSet.includes(T_i) && targetSet.includes(T_j)){
			// B_A matrix
			var B_A = [[agents[A_a].sensingRate-targets[T_i].uncertaintyRate, -targets[T_i].uncertaintyRate],[-targets[T_j].uncertaintyRate, agents[A_a].sensingRate-targets[T_j].uncertaintyRate]];
			var A = [[targets[T_i].uncertaintyRate],[targets[T_j].uncertaintyRate]];
			var t_cy = 2*paths[p].distPath()/agents[A_a].maxLinearVelocity;
			
			//print(t_cy);
			var dwellTimes = math.multiply(math.inv(B_A),math.multiply(A,t_cy));

			var BMinusA = [[agents[A_a].sensingRate-targets[T_i].uncertaintyRate],[agents[A_a].sensingRate-targets[T_j].uncertaintyRate]];
			var meanUncertainty = 0.5*math.multiply(math.transpose(BMinusA),dwellTimes);
			print("Path between T_"+(T_i+1)+","+(T_j+1)+": "+meanUncertainty.toFixed(2));

			var neglectUncertainty1 = targets[T_i].initialUncertainty + 0.5*targets[T_i].uncertaintyRate*periodT;
			var neglectUncertainty2 = targets[T_j].initialUncertainty + 0.5*targets[T_j].uncertaintyRate*periodT;
			var meanUncertaintyIfNotCovered = neglectUncertainty1 + neglectUncertainty2; 

			print("Uncovered: "+meanUncertaintyIfNotCovered);
			var gain = meanUncertaintyIfNotCovered - meanUncertainty;

			if(gain>maxGain){
				maxGain = gain;
				bestPath = p;
			}
		}
	
	}

	var pathsInTour = [bestPath,bestPath];
	var targetsInTour = paths[bestPath].targets;
	var costOfTour = maxGain;

	return [pathsInTour,targetsInTour,costOfTour];

}


function getBestTargetToAdd(agentID,targetSet,pathsInTour,targetsInTour,costOfTour){

	// target list which are not in the path
	var candidateTargets = [];
	for(var i = 0; i<targetSet.length; i++){
		if(!targetsInTour.includes(targetSet[i])){
			candidateTargets.push(targetSet[i]);
		}
	}
	print("Search space: T:"+candidateTargets);


	var maxGain = -1;
	var bestTour = [];
	
	for(var cT = 0; cT<candidateTargets.length; cT++){
		// for each candidate target, need to find the best path to remove from the existing tour
		// to maximize the gain of adding the candidate target;
		
		var T_c = candidateTargets[cT];
		print("T_c:"+T_c);

		var meanUncertaintyIfNotCovered = targets[T_c].initialUncertainty + 0.5*targets[T_c].uncertaintyRate*periodT;

		for(var p = 0; p<pathsInTour.length; p++){ // path to be removed so that T_c can be added via two new paths
			var pID = pathsInTour[p];
			var newPathsInTour = [...pathsInTour];
			var newTargetsInTour = [...targetsInTour];
			if(targetsInTour[p]==paths[pID].targets[0]){
				var newPath1 = getPathID(paths[pID].targets[0],T_c);
				var newPath2 = getPathID(paths[pID].targets[1],T_c);
			}else{
				var newPath1 = getPathID(paths[pID].targets[1],T_c);
				var newPath2 = getPathID(paths[pID].targets[0],T_c);
			}
			////print("Paths: "+newPathsInTour);
			//print("Targets: "+newTargetsInTour);
			newPathsInTour.splice(p,1,newPath1,newPath2);// replace the element at index p with
			newTargetsInTour.splice(p+1,0,T_c);
			//print("Targets: "+newTargetsInTour);
			meanUncertaintyOfTheTour = getMeanUncertainty(agentID,newPathsInTour,newTargetsInTour);
			var gain = costOfTour - meanUncertaintyOfTheTour + 	meanUncertaintyIfNotCovered;
			//////print("gain:"+gain+", max: "+maxGain);
			if(gain>maxGain){
				maxGain	= gain;
				bestTour = [newPathsInTour,newTargetsInTour,meanUncertaintyOfTheTour];
			}	
			

		}

	}

	print("Result: "+bestTour[1]);
	return bestTour;	

}


function getMeanUncertainty(agentID,pathsInTour,targetsInTour){

	var B_A = [];
	var A = [];
	var dist = 0;
	var BMinusA	= [];
	for(var i = 0; i<targetsInTour.length; i++){
		T_i = targetsInTour[i];
		B_A.push([]);// new row
		for(var j = 0; j<targetsInTour.length; j++){
			T_j = targetsInTour[j];
			if(i==j){
				B_A[i].push(agents[agentID].sensingRate-targets[T_i].uncertaintyRate);
			}else{
				B_A[i].push(-targets[T_i].uncertaintyRate);
			}
		}
		
		A.push([targets[T_i].uncertaintyRate]);
		dist = dist + paths[pathsInTour[i]].distPath();
		BMinusA.push([agents[agentID].sensingRate-targets[T_i].uncertaintyRate]);
	}
	//print("B_A="+B_A);
	var t_cy = dist/agents[agentID].maxLinearVelocity;
	//print(t_cy);
	var dwellTimes = math.multiply(math.inv(B_A),math.multiply(A,t_cy));
	var meanUncertainty = 0.5*math.multiply(math.transpose(BMinusA),dwellTimes);
	print("Targets: "+targetsInTour+": "+meanUncertainty.toFixed(2));

	return meanUncertainty;
}

function getPathID(targetID1,targetID2){
	for(var i = 0; i<paths.length; i++){
		if(paths[i].targets.includes(targetID1) && paths[i].targets.includes(targetID2)){
			return i;
		}
	}
	print("Error - path finding, between T_"+targetID1+","+targetID2);
}






function swap2OPT(route,i,k,agentID){

	var swappedRoute = [];
	var pathList = [];
	var targetList = [];


	for(var p = 0; p<i; p++){
		pathList[p] = route[0][p];
		targetList[p] = route[1][p];
	}
	pathList[i] = getPathID(route[1][i],route[1][k]);
	targetList[i] = route[1][i];
	
	count = 1;
	for(var p = i+1; p<k; p++){
		pathList[p] = route[0][k-count];
		targetList[p] = route[1][k-count+1];
		count++;
	}
	if(k==(route[0].length-1)){
		pathList[k] = getPathID(route[1][i+1],route[1][0]);
	}else{
		pathList[k] = getPathID(route[1][i+1],route[1][k+1]);
	}
	targetList[k] = route[1][i+1];

	for(var p = k+1; p<route[0].length; p++){
		pathList[p] = route[0][p];
		targetList[p] = route[1][p];
	}

	//print("Targets: "+route[1]);
	//print("swapped Targets: "+targetList);
	var cost = getMeanUncertainty(agentID,pathList,targetList);
	if(cost<route[2]){
		print("Paths: "+route[0]);
		print("swapped Paths: "+pathList);
		
		print("Cost: "+route[2]);
		print("Swapped Cost: "+cost);

		consolePrint("Steady state mean uncertainty reduced by "+(route[2]-cost).toFixed(3)+" (to "+cost.toFixed(3)+") via 2-OPT method.");	
		route = [pathList,targetList,cost]
	} 
	return route;
	//print("Swapped cost: "+cost);
		
}


function swap3OPT(route,i,j,k,agentID){

	// Type 0 - nominal - cost = route[2]
	minRouteSwap = [...route]; 
	minCost = route[2];

	// Type 1
	var swappedRoute = [];
	var pathList = [];
	var targetList = [];

	for(var p = 0; p<i; p++){
		pathList[p] = route[0][p];
		targetList[p] = route[1][p];
	}
	pathList[i] = getPathID(route[1][i],route[1][j]);
	targetList[i] = route[1][i];
	
	
	count = 1;
	for(var p = i+1; p<j; p++){
		pathList[p] = route[0][j-count];
		targetList[p] = route[1][j-count+1];
		count++;
	}
	pathList[j] = getPathID(route[1][k],route[1][i+1]);
	targetList[j] = route[1][i+1];

	
	count = 1;
	for(var p = j+1; p<k; p++){
		pathList[p] = route[0][k-count];
		targetList[p] = route[1][k-count+1];
		count++;
	}	
	if(k==(route[0].length-1)){
		pathList[k] = getPathID(route[1][j+1],route[1][0]);
	}else{
		pathList[k] = getPathID(route[1][j+1],route[1][k+1]);
	}
	targetList[k] = route[1][j+1];


	for(var p = k+1; p<route[0].length; p++){
		pathList[p] = route[0][p];
		targetList[p] = route[1][p];
	}

	print("Targets: "+route[1]);
	print("Paths: "+route[0]);
	print("swapped1 Targets: "+targetList);
	print("swapped1 Paths: "+pathList);
	var cost = getMeanUncertainty(agentID,pathList,targetList);
	if(cost<minCost){
		minCost	= cost;
		print("Prev cost: "+route[2]);
		print("Swapped1 Cost: "+cost);
		minRouteSwap = [pathList,targetList,cost]
	} 
	//for:1,0,4,2,3 -- under i,j,k= 0,2,4
	//1-4-0-3-2 --- 1



	// Type 2
	var swappedRoute = [];
	var pathList = [];
	var targetList = [];

	for(var p = 0; p<i; p++){
		pathList[p] = route[0][p];
		targetList[p] = route[1][p];
	}
	pathList[i] = getPathID(route[1][i],route[1][j+1]);
	targetList[i] = route[1][i];
	
	
	count = 1;
	for(var p = j+1; p<k; p++){
		pathList[i+count] = route[0][p];
		targetList[i+count] = route[1][p];
		count++;
	}
	pathList[i+count] = getPathID(route[1][k],route[1][i+1]);
	targetList[i+count] = route[1][k];
	count++;
	
	for(var p = i+1; p<j; p++){
		pathList[i+count] = route[0][p];
		targetList[i+count] = route[1][p];
		count++;
	}	
	if(k==(route[0].length-1)){
		pathList[i+count] = getPathID(route[1][j],route[1][0]);
	}else{
		pathList[i+count] = getPathID(route[1][j],route[1][k+1]);
	}
	targetList[i+count] = route[1][j];

	print("check: i+count="+(i+count)+", k="+k);
	for(var p = k+1; p<route[0].length; p++){
		pathList[p] = route[0][p];
		targetList[p] = route[1][p];
	}

	print("Targets: "+route[1]);
	print("Paths: "+route[0]);
	print("swapped2 Targets: "+targetList);
	print("swapped2 Paths: "+pathList);
	// 1-2-3-0-4-
	var cost = getMeanUncertainty(agentID,pathList,targetList);
	if(cost<minCost){
		minCost	= cost;
		print("Prev cost: "+route[2]);
		print("Swapped2 Cost: "+cost);
		minRouteSwap = [pathList,targetList,cost]
	} 




	// Type 3
	var swappedRoute = [];
	var pathList = [];
	var targetList = [];

	for(var p = 0; p<i; p++){
		pathList[p] = route[0][p];
		targetList[p] = route[1][p];
	}
	pathList[i] = getPathID(route[1][i],route[1][k]);
	targetList[i] = route[1][i];
	
	
	count = 1;
	for(var p = i+1; p<i+(k-j); p++){
		pathList[p] = route[0][k-count];
		targetList[p] = route[1][k-count+1];
		count++;
	}
	pathList[p] = getPathID(route[1][j+1],route[1][i+1]);
	print("special: "+pathList[p]+", p="+p)
	targetList[p] = route[1][j+1];
	count ++;

	for(var p = i+1; p<j; p++){
		pathList[i+count] = route[0][p];
		targetList[i+count] = route[1][p];
		count++;
	}	
	if(k==(route[0].length-1)){
		pathList[k] = getPathID(route[1][0],route[1][j]);
	}else{
		pathList[k] = getPathID(route[1][k+1],route[1][j]);
	}
	targetList[k] = route[1][j];


	for(var p = k+1; p<route[0].length; p++){
		pathList[p] = route[0][p];
		targetList[p] = route[1][p];
	}

	print("Targets: "+route[1]);
	print("swapped3 Targets: "+targetList);
	print("Paths: "+route[0]);
	print("swapped3 Paths: "+pathList);
		//1-3-2-0-4
	var cost = getMeanUncertainty(agentID,pathList,targetList);
	if(cost<minCost){
		minCost	= cost;
		print("Prev cost: "+route[2]);
		print("Swapped3 Cost: "+cost);
		minRouteSwap = [pathList,targetList,cost]
	} 

	if(minCost!=route[2]){
		consolePrint("Steady state mean uncertainty reduced by "+(route[2]-minCost).toFixed(3)+" (to "+minCost.toFixed(3)+") via 3-OPT method.");
		print("Cost improved from 3-OPT!!!");	
	}

	return	minRouteSwap;
		
}



// boosting

// threshold based on TSP
function generateThresholdsFromRoute(){
    // biase the threshods
    var agentID = 0;
    if(cyclicRoutes.length==0){
    	consolePrint("Generate initial trajectory first (Use 'Search' button)!!!");
    	return;
    }
    var route = cyclicRoutes[agentID];
    
    var pathList = route[0];
    var targetList = route[1];

   	var z = agentID;
    for(var p = 0; p < targets.length; p++){
        for(var q = 0; q < targets.length; q++){
            
            if(p==q){// policy
            	agents[z].threshold[p][q] = 0;
            }else{
            	var path_pq = getPathID(p,q);
            	if(!pathList.includes(path_pq) && paths[path_pq].isPermenent){
            		agents[z].threshold[p][q] = 100;	
            	}else{
            		var T_pIndex = pathList.indexOf(path_pq);
            		var T_qIndex;
            		if(T_pIndex==pathList.length-1){
            			var T_qIndex = 0;
            		}else{
            			var T_qIndex = T_pIndex+1;
            		}

            		if(targetList[T_pIndex]==p && targetList[T_qIndex]==q){
            			agents[z].threshold[p][q] = 0;		
            		}else if(!paths[path_pq].isPermenent){
            			agents[z].threshold[p][q] = 10000;	
            		}else{
            			agents[z].threshold[p][q] = 100;		
            		}
            	}
            } 
        }
    }
    

    displayThresholds();
}

// mild perturbation
function addRandomNoiseToThresholds(){
    

   	for(var z = 0; z<agents.length; z++){
	    for(var p = 0; p < targets.length; p++){
	        for(var q = 0; q < targets.length; q++){
	            var path_pq = getPathID(p,q);
	            if(p==q){// policy
	            	agents[z].threshold[p][q] = agents[z].threshold[p][q] + randomNoiseLevelForThresholds*Math.random();
	            }else if(!paths[path_pq].isPermenent){
	            	agents[z].threshold[p][q] =  10000;
	            }else{
	            	agents[z].threshold[p][q] = agents[z].threshold[p][q] - 3*randomNoiseLevelForThresholds*Math.random();
	            	if(agents[z].threshold[p][q]<0){
	            		agents[z].threshold[p][q] = randomNoiseLevelForThresholds*Math.random();
	            	}
	            } 
	        }
	    }
    }

    displayThresholds();
}


// fit cycles optimize and randomize again
function identifyCycleAndOptimize(){
    
}


