
function Cycle(agentID,targetSet){
	

	this.deployedAgent = agentID; // agent which uses this cycle
	this.allowedTargetList = targetSet; // cycle is constrained on this target set

	this.pathList = []; // path ids of each link
	this.targetList = []; // target id's of each target
	
	this.meanUncertainty = -1; // mean uncertainty vallue
	this.meanUncertaintyGain = -1; // meanUncertainty improvement due to the presence of the cycle (//meanUncertaintyIfNotCovered - meanUncertainty;)



	this.show = function(){
		var colorValue = color(200,0,0,128);
		for(var i = 0; i<this.pathList.length; i++){
			paths[this.pathList[i]].highlight(colorValue);
		}
	}

	this.highlight = function(){
		var colorValue = color(0,200,0,128);
		for(var i = 0; i<this.pathList.length; i++){
			paths[this.pathList[i]].highlight(colorValue);
		}
	}	


	this.clone = function(){
		var resultCycle = new Cycle(this.deployedAgent,this.allowedTargetList);
		resultCycle.pathList = [...this.pathList];
		resultCycle.targetList = [...this.targetList];
		resultCycle.meanUncertainty = this.meanUncertainty;
		resultCycle.meanUncertaintyGain = this.meanUncertaintyGain;
		return resultCycle;
	}


	this.cloneFrom = function(givenCycle){
		this.deployedAgent = givenCycle.deployedAgent; 
		this.allowedTargetList = givenCycle.allowedTargetList; 
		this.pathList = [...givenCycle.pathList];
		this.targetList = [...givenCycle.targetList];
		this.meanUncertainty = givenCycle.meanUncertainty;
		this.meanUncertaintyGain = givenCycle.meanUncertaintyGain;

	}



	this.computeBestInitialCycle = function(){// to initialize the  problem

		var maxGain = -1;
		var minMeanUncertainty = -1;
		var bestPath = -1;

		// path means we are adding two targets to the monitoring list
		for(var p = 0; p<paths.length; p++){

			var T_i = paths[p].targets[0];
			var T_j = paths[p].targets[1];
			var A_a = this.deployedAgent;

			if(this.allowedTargetList.includes(T_i) && this.allowedTargetList.includes(T_j)){
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
					minMeanUncertainty = meanUncertainty;
				}
			}
		
		}

		this.pathList = [bestPath,bestPath];
		this.targetList = paths[bestPath].targets;
		this.meanUncertainty = minMeanUncertainty;
		this.meanUncertaintyGain = maxGain;
		/*return [pathsInTour,targetsInTour,costOfTour];*/
	}



	this.addTheBestAvailableTarget = function(){

		//arguments agentID,targetSet,pathsInTour,targetsInTour,costOfTour

		// target list which are not in the path
		var candidateTargets = [];
		for(var i = 0; i<this.allowedTargetList.length; i++){
			if(!this.targetList.includes(this.allowedTargetList[i])){
				candidateTargets.push(this.allowedTargetList[i]);
			}
		}
		print("Search space: T:"+candidateTargets);
		// end finding search space targets


		var maxGain = -1;
		var minMeanUncertainty = -1;
		////var bestTour = [];
		var bestTour = -1;
		var dummyTour = this.clone();
		
		for(var cT = 0; cT<candidateTargets.length; cT++){
			// for each candidate target, need to find the best path to remove from the existing tour
			// to maximize the gain of adding the candidate target;
			
			var T_c = candidateTargets[cT];
			print("T_c:"+T_c);

			var meanUncertaintyIfNotCovered = targets[T_c].initialUncertainty + 0.5*targets[T_c].uncertaintyRate*periodT;

			for(var p = 0; p < this.pathList.length; p++){ // path to be removed so that T_c can be added via two new paths
				var pID = this.pathList[p];

				dummyTour = this.clone();
				if(this.targetList[p]==paths[pID].targets[0]){
					var newPath1 = getPathID(paths[pID].targets[0],T_c);
					var newPath2 = getPathID(paths[pID].targets[1],T_c);
				}else{
					var newPath1 = getPathID(paths[pID].targets[1],T_c);
					var newPath2 = getPathID(paths[pID].targets[0],T_c);
				}
				////print("Paths: "+newPathsInTour);
				//print("Targets: "+newTargetsInTour);
				dummyTour.pathList.splice(p,1,newPath1,newPath2);// replace the element at index p with
				dummyTour.targetList.splice(p+1,0,T_c);
				dummyTour.computeMeanUncertainty(); // update the meanUncertainty and mean uncertainty gain
				//print("Targets: "+newTargetsInTour);
				var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
				var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
				//////print("gain:"+gain+", max: "+maxGain);
				if(gain>maxGain){
					maxGain	= gain;
					
					//bestTour = [newPathsInTour,newTargetsInTour,meanUncertaintyOfTheTour];
					bestTour = dummyTour.clone();
		
				}	
				

			}

		}

		
		if(bestTour==-1){
			print("No way to improve!");
			print("Final Cycle: "+this.targetList+", cost: "+this.meanUncertainty.toFixed(2));
			return -1;
		}else{
			print("Result: "+bestTour.targetList);	
			this.cloneFrom(bestTour);
			return +1;	
		}

	}



	this.computeMeanUncertainty = function(){

		// to compute the mean uncertainty gain of the cycle (i.e. meanuncertainty diffrence{notcovered-coverd})
		var meanUncertaintyIfNotCovered = 0;

		// agentID,pathsInTour,targetsInTour
		var B_A = [];
		var A = [];
		var dist = 0;
		var BMinusA	= [];

		for(var i = 0; i < this.targetList.length; i++){
			
			T_i = this.targetList[i];
			B_A.push([]);// new row
			for(var j = 0; j<this.targetList.length; j++){
				T_j = this.targetList[j];
				if(i==j){
					B_A[i].push(agents[this.deployedAgent].sensingRate-targets[T_i].uncertaintyRate);
				}else{
					B_A[i].push(-targets[T_i].uncertaintyRate);
				}
			}
			
			A.push([targets[T_i].uncertaintyRate]);
			dist = dist + paths[this.pathList[i]].distPath();
			BMinusA.push([agents[this.deployedAgent].sensingRate-targets[T_i].uncertaintyRate]);
		
			meanUncertaintyIfNotCovered = meanUncertaintyIfNotCovered + targets[T_i].initialUncertainty + 0.5*targets[T_i].uncertaintyRate*periodT;

		}
		//print("B_A="+B_A);
		var t_cy = dist/agents[this.deployedAgent].maxLinearVelocity;
		//print(t_cy);
		var dwellTimes = math.multiply(math.inv(B_A),math.multiply(A,t_cy));
		var meanUncertainty = 0.5*math.multiply(math.transpose(BMinusA),dwellTimes);
		var meanUncertaintyGain = meanUncertaintyIfNotCovered - meanUncertainty;

		print("Targets: "+this.targetList+": "+meanUncertainty.toFixed(2));

		this.meanUncertainty = meanUncertainty;
		this.meanUncertaintyGain = meanUncertaintyGain;
		//return meanUncertainty;
	}



	this.swap2OPT = function(i,k){
		
		////var pathList = [];
		////var targetList = [];
		var dummyTour = new Cycle(this.deployedAgent,this.allowedTargetList);
		
		for(var p = 0; p<i; p++){
			dummyTour.pathList[p] = this.pathList[p];
			dummyTour.targetList[p] = this.targetList[p];
		}
		dummyTour.pathList[i] = getPathID(this.targetList[i],this.targetList[k]);
		dummyTour.targetList[i] = this.targetList[i];
		
		count = 1;
		for(var p = i+1; p<k; p++){
			dummyTour.pathList[p] = this.pathList[k-count];
			dummyTour.targetList[p] = this.targetList[k-count+1];
			count++;
		}
		if(k==(this.pathList.length-1)){
			dummyTour.pathList[k] = getPathID(this.targetList[i+1],this.targetList[0]);
		}else{
			dummyTour.pathList[k] = getPathID(this.targetList[i+1],this.targetList[k+1]);
		}
		dummyTour.targetList[k] = this.targetList[i+1];

		for(var p = k+1; p<this.pathList.length; p++){
			dummyTour.pathList[p] = this.pathList[p];
			dummyTour.targetList[p] = this.targetList[p];
		}

		//print("Targets: "+this.dummyTour.targetList);
		//print("swapped Targets: "+dummyTour.targetList);
		dummyTour.computeMeanUncertainty();
		//var cost = getMeanUncertainty(agentID,dummyTour.pathList,dummyTour.targetList);
		var cost = dummyTour.meanUncertainty;
		
		if(cost<this.meanUncertainty){
			print("Paths: "+this.pathList);
			print("swapped Paths: "+dummyTour.pathList);
			
			print("Cost: "+this.meanUncertainty);
			print("Swapped Cost: "+cost);

			consolePrint("Steady state mean uncertainty reduced by "+(this.meanUncertainty-cost).toFixed(3)+" (to "+cost.toFixed(3)+") via 2-OPT method.");	
			this.cloneFrom(dummyTour);
			////route = [dummyTour.pathList,dummyTour.targetList,cost]
		} 

		dummyTour.highlight();
		//print("Swapped cost: "+cost);
			
	}



	this.swap3OPT = function(i,j,k,l){

		// args : route,i,j,k,agentID

		// Type 1 - nominal - cost = this.meanUncertainty
		if(l==0){
			////minRouteSwap = [...route]; 
			minCost = this.meanUncertainty;

			// Type 1
			////var pathList = [];
			////var targetList = [];
			var dummyTour = new Cycle(this.deployedAgent,this.allowedTargetList);

			for(var p = 0; p<i; p++){
				dummyTour.pathList[p] = this.pathList[p];
				dummyTour.targetList[p] = this.targetList[p];
			}
			dummyTour.pathList[i] = getPathID(this.targetList[i],this.targetList[j]);
			dummyTour.targetList[i] = this.targetList[i];
			
			
			count = 1;
			for(var p = i+1; p<j; p++){
				dummyTour.pathList[p] = this.pathList[j-count];
				dummyTour.targetList[p] = this.targetList[j-count+1];
				count++;
			}
			dummyTour.pathList[j] = getPathID(this.targetList[k],this.targetList[i+1]);
			dummyTour.targetList[j] = this.targetList[i+1];

			
			count = 1;
			for(var p = j+1; p<k; p++){
				dummyTour.pathList[p] = this.pathList[k-count];
				dummyTour.targetList[p] = this.targetList[k-count+1];
				count++;
			}	
			if(k==(this.pathList.length-1)){
				dummyTour.pathList[k] = getPathID(this.targetList[j+1],this.targetList[0]);
			}else{
				dummyTour.pathList[k] = getPathID(this.targetList[j+1],this.targetList[k+1]);
			}
			dummyTour.targetList[k] = this.targetList[j+1];


			for(var p = k+1; p<this.pathList.length; p++){
				dummyTour.pathList[p] = this.pathList[p];
				dummyTour.targetList[p] = this.targetList[p];
			}

			print("Targets: "+this.targetList);
			print("Paths: "+this.pathList);
			print("swapped1 Targets: "+dummyTour.targetList);
			print("swapped1 Paths: "+dummyTour.pathList);

			dummyTour.computeMeanUncertainty();
			////var cost = getMeanUncertainty(this.deployedAgent,dummyTour.pathList,dummyTour.targetList);
			var cost = dummyTour.meanUncertainty;
			if(cost<minCost){
				minCost	= cost;
				print("Prev cost: "+this.meanUncertainty);
				print("Swapped1 Cost: "+cost);

				consolePrint("Steady state mean uncertainty reduced by "+(this.meanUncertainty-minCost).toFixed(3)+" (to "+minCost.toFixed(3)+") via 3-OPT method.");
				print("Cost improved from 3-OPT!!!");

				this.cloneFrom(dummyTour);
				////minRouteSwap = [dummyTour.pathList,dummyTour.targetList,cost]
			} 
			//for:1,0,4,2,3 -- under i,j,k= 0,2,4
			//1-4-0-3-2 --- 1

			dummyTour.highlight();
		

		}else if(l==1){

			// Type 2
			
			////var pathList = [];
			////var targetList = [];
			var dummyTour = new Cycle(this.deployedAgent,this.allowedTargetList);

			for(var p = 0; p<i; p++){
				dummyTour.pathList[p] = this.pathList[p];
				dummyTour.targetList[p] = this.targetList[p];
			}
			dummyTour.pathList[i] = getPathID(this.targetList[i],this.targetList[j+1]);
			dummyTour.targetList[i] = this.targetList[i];
			
			
			count = 1;
			for(var p = j+1; p<k; p++){
				dummyTour.pathList[i+count] = this.pathList[p];
				dummyTour.targetList[i+count] = this.targetList[p];
				count++;
			}
			dummyTour.pathList[i+count] = getPathID(this.targetList[k],this.targetList[i+1]);
			dummyTour.targetList[i+count] = this.targetList[k];
			count++;
			
			for(var p = i+1; p<j; p++){
				dummyTour.pathList[i+count] = this.pathList[p];
				dummyTour.targetList[i+count] = this.targetList[p];
				count++;
			}	
			if(k==(this.pathList.length-1)){
				dummyTour.pathList[i+count] = getPathID(this.targetList[j],this.targetList[0]);
			}else{
				dummyTour.pathList[i+count] = getPathID(this.targetList[j],this.targetList[k+1]);
			}
			dummyTour.targetList[i+count] = this.targetList[j];

			print("check: i+count="+(i+count)+", k="+k);
			for(var p = k+1; p<this.pathList.length; p++){
				dummyTour.pathList[p] = this.pathList[p];
				dummyTour.targetList[p] = this.targetList[p];
			}

			print("Targets: "+this.targetList);
			print("Paths: "+this.pathList);
			print("swapped2 Targets: "+dummyTour.targetList);
			print("swapped2 Paths: "+dummyTour.pathList);
			// 1-2-3-0-4-

			dummyTour.computeMeanUncertainty();
			////var cost = getMeanUncertainty(this.deployedAgent,dummyTour.pathList,dummyTour.targetList);
			var cost = dummyTour.meanUncertainty; 
			if(cost<minCost){
				minCost	= cost;
				print("Prev cost: "+this.meanUncertainty);
				print("Swapped2 Cost: "+cost);

				consolePrint("Steady state mean uncertainty reduced by "+(this.meanUncertainty-minCost).toFixed(3)+" (to "+minCost.toFixed(3)+") via 3-OPT method.");
				print("Cost improved from 3-OPT!!!");

				this.cloneFrom(dummyTour);
				////minRouteSwap = [dummyTour.pathList,dummyTour.targetList,cost]
			} 
			dummyTour.highlight();


		}else if(l==2){

			// Type 3
			
			////var pathList = [];
			////var targetList = [];
			var dummyTour = new Cycle(this.deployedAgent,this.allowedTargetList);

			for(var p = 0; p<i; p++){
				dummyTour.pathList[p] = this.pathList[p];
				dummyTour.targetList[p] = this.targetList[p];
			}
			dummyTour.pathList[i] = getPathID(this.targetList[i],this.targetList[k]);
			dummyTour.targetList[i] = this.targetList[i];
			
			
			count = 1;
			for(var p = i+1; p<i+(k-j); p++){
				dummyTour.pathList[p] = this.pathList[k-count];
				dummyTour.targetList[p] = this.targetList[k-count+1];
				count++;
			}
			dummyTour.pathList[p] = getPathID(this.targetList[j+1],this.targetList[i+1]);
			print("special: "+dummyTour.pathList[p]+", p="+p)
			dummyTour.targetList[p] = this.targetList[j+1];
			count ++;

			for(var p = i+1; p<j; p++){
				dummyTour.pathList[i+count] = this.pathList[p];
				dummyTour.targetList[i+count] = this.targetList[p];
				count++;
			}	
			if(k==(this.pathList.length-1)){
				dummyTour.pathList[k] = getPathID(this.targetList[0],this.targetList[j]);
			}else{
				dummyTour.pathList[k] = getPathID(this.targetList[k+1],this.targetList[j]);
			}
			dummyTour.targetList[k] = this.targetList[j];


			for(var p = k+1; p<this.pathList.length; p++){
				dummyTour.pathList[p] = this.pathList[p];
				dummyTour.targetList[p] = this.targetList[p];
			}

			print("Targets: "+this.targetList);
			print("swapped3 Targets: "+dummyTour.targetList);
			print("Paths: "+this.pathList);
			print("swapped3 Paths: "+dummyTour.pathList);
				//1-3-2-0-4
			
			dummyTour.computeMeanUncertainty();
			////var cost = getMeanUncertainty(this.deployedAgent,dummyTour.pathList,dummyTour.targetList);
			var cost = dummyTour.meanUncertainty;
			if(cost<minCost){
				minCost	= cost;
				print("Prev cost: "+this.meanUncertainty);
				print("Swapped3 Cost: "+cost);

				consolePrint("Steady state mean uncertainty reduced by "+(this.meanUncertainty-minCost).toFixed(3)+" (to "+minCost.toFixed(3)+") via 3-OPT method.");
				print("Cost improved from 3-OPT!!!");	

				this.cloneFrom(dummyTour);
				////minRouteSwap = [dummyTour.pathList,dummyTour.targetList,cost]
				
			} 
			dummyTour.highlight();

		}




		// if(minCost!=this.meanUncertainty){
		// 	consolePrint("Steady state mean uncertainty reduced by "+(this.meanUncertainty-minCost).toFixed(3)+" (to "+minCost.toFixed(3)+") via 3-OPT method.");
		// 	print("Cost improved from 3-OPT!!!");	
		// }

		// return	minRouteSwap;
			
	}


	// threshold based on TSP
	this.computeThresholds = function(){
	    // biase the threshods
	    var agentID = this.deployedAgent;
	    if(cycles.length==0){
	    	consolePrint("Generate initial trajectory first (Use 'Search' button)!!!");
	    	return;
	    }

	    	    
	    var pathList = this.pathList;
	    var targetList = this.targetList;

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




}




function initiateComputingRefinedGreedyCycles(){
	var agentID	= 0;
	var targetSet = math.range(0,targets.length)._data;

	cycles.push(new Cycle(agentID,targetSet)); //creating a cycle for the agent (with agent id) on the target set
	
	cycles[0].computeBestInitialCycle();
	RGCComputingMode = 1; //start iteratively evolving the cycles
}

function iterationOfComputingGreedyCycles(){
	print("Current route length: "+cycles[0].targetList.length+" targets.");
	if(RGCComputingMode==1){//initial greedy
		sleepFor(100);
		var val = cycles[0].addTheBestAvailableTarget();
		if(val==-1){
			consolePrint("Steady state mean uncertainty (J) ="+cycles[0].meanUncertainty.toFixed(3)+", achieved via greedy cycle search method.");
			RGCComputingMode = 2;
			cycleRefiningParameters[0] = [0,2]; //[i,k] for 2-opt
			sleepFor(500);
		}
		
	}else if(RGCComputingMode==2){//2-opt
		var i = cycleRefiningParameters[0][0];
		var k = cycleRefiningParameters[0][1];
		sleepFor(100);
		cycles[0].swap2OPT(i,k);
		
		k = k + 1;
		if(k >= cycles[0].pathList.length){
			i = i + 1;
			if(i >= cycles[0].pathList.length-2){
				consolePrint("2-Opt refining stage finished!");
				RGCComputingMode = 3;
				cycleRefiningParameters[1] = [0,1,2,0]; // i,j,k,l for 3-Opt
				sleepFor(500);
			}
			k = i + 2;
		}
		cycleRefiningParameters[0][0] = i;
		cycleRefiningParameters[0][1] = k;
	
	}else if(RGCComputingMode==3){//3-opt

		var i = cycleRefiningParameters[1][0];
		var j = cycleRefiningParameters[1][1];
		var k = cycleRefiningParameters[1][2];
		var l = cycleRefiningParameters[1][3];
		sleepFor(100);
		cycles[0].swap3OPT(i,j,k,l);
		
		l = l + 1;
		if(l >= 3){	
			k = k + 1;
			if(k >= cycles[0].pathList.length){
				j = j + 1;
				if(j >= cycles[0].pathList.length-1){
					i = i + 1;
					if(i >= cycles[0].pathList.length-2){
						consolePrint("3-Opt refining stage finished!");
						RGCComputingMode = 0;
						sleepFor(500);
					}
					j = i + 1;
				}
				k = j + 1;
			}
			l = 0;
		}
		cycleRefiningParameters[1][0] = i;
		cycleRefiningParameters[1][1] = j;
		cycleRefiningParameters[1][2] = k;
		cycleRefiningParameters[1][3] = l;

	}
}


function resetCycles(){
	RGCComputingMode = 0;
	for(var i = cycles.length; i > 0; i--){
        removeACycle();
    }
}


function generateThresholdsFromRoutes(){
	for(var i = 0; i<cycles.length; i++){
		cycles[i].computeThresholds();
	}
}




// greedy construction of optimal cycle to travel.- for one agent

function computeRefinedGreedyCycles(){
	
	var agentID	= 0;
	var targetSet = math.range(0,targets.length)._data;

	cycles.push(new Cycle(agentID,targetSet)); //creating a cycle for the agent (with agent id) on the target set
	cycles[0].computeBestInitialCycle();



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



function removeACycle(){
	cycles.pop();
}



// fit cycles optimize and randomize again
function identifyCycleAndOptimize(){
    
}