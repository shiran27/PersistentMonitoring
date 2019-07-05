
function Cycle(agentID,targetSet){
	

	this.deployedAgent = agentID; // agent which uses this cycle
	this.allowedTargetList = targetSet; // cycle is constrained on this target set

	this.pathList = []; // path ids of each link
	this.targetList = []; // target id's of each target
	
	this.meanUncertainty = -1; // mean uncertainty vallue

	this.travelTimeList = []; // travel time between adjadcent targets
	this.dwellTimeList = []; // dwell time between adjadcent targets
	this.subCycleList = []; // array of arrays: L x L : 1's and 0's 
	this.subCycleTravelTimeList = []; // M x 1 : t_ij values
	this.subCycleDwellTimeList = []; 

	this.RGCComputingMode = 0;
	this.cycleRefiningParameters = [];

	this.show = function(){
		var colorValue = color(200,0,0,128);
		for(var i = 0; i<this.pathList.length; i++){
			paths[this.pathList[i]].highlight(colorValue);
		}
	}

	this.highlight = function(){
		if(this.RGCComputingMode==2){
			var colorValue = color(0,200,0,128);
		}else if(this.RGCComputingMode==3){
			var colorValue = color(0,0,200,128);
		}else{
			//var colorValue = color(0,200,200,128);
		}



		for(var i = 0; i<this.pathList.length; i++){
			paths[this.pathList[i]].highlight(colorValue);
		}
	}	


	this.clone = function(){
		var resultCycle = new Cycle(this.deployedAgent,this.allowedTargetList);
		resultCycle.pathList = [...this.pathList];
		resultCycle.targetList = [...this.targetList];
		resultCycle.meanUncertainty = this.meanUncertainty;

		resultCycle.travelTimeList = [...this.travelTimeList];
		resultCycle.dwellTimeList = [...this.dwellTimeList];
		resultCycle.subCycleList = [...this.subCycleList];
		resultCycle.subCycleTravelTimeList = [...this.subCycleTravelTimeList];
		resultCycle.subCycleDwellTimeList = [...this.subCycleDwellTimeList];
		
		resultCycle.cycleRefiningParameters = [...this.cycleRefiningParameters];
		resultCycle.RGCComputingMode = this.RGCComputingMode;
		
		return resultCycle;
	}


	this.cloneFrom = function(givenCycle){
		this.deployedAgent = givenCycle.deployedAgent; 
		this.allowedTargetList = givenCycle.allowedTargetList; 
		this.pathList = [...givenCycle.pathList];
		this.targetList = [...givenCycle.targetList];
		this.meanUncertainty = givenCycle.meanUncertainty;
		
		this.travelTimeList = [...givenCycle.travelTimeList];
		this.dwellTimeList = [...givenCycle.dwellTimeList];
		this.subCycleList = [...givenCycle.subCycleList];
		this.subCycleTravelTimeList = [...givenCycle.subCycleTravelTimeList];
		this.subCycleDwellTimeList = [...givenCycle.subCycleDwellTimeList];
		
		this.cycleRefiningParameters = [...givenCycle.cycleRefiningParameters];
		this.RGCComputingMode = givenCycle.RGCComputingMode;
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

			if(this.allowedTargetList.includes(T_i) && this.allowedTargetList.includes(T_j) && paths[p].isPermenent){
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

		if(bestPath != -1){
			this.pathList = [bestPath,bestPath];
			this.targetList = [...paths[bestPath].targets];
			this.meanUncertainty = minMeanUncertainty;
		}else{
			this.pathList = [];
			this.targetList = [this.allowedTargetList[0]];
			this.meanUncertainty = 0;
		}
	}



	this.addTheBestAvailableTarget = function(){

		
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

				
				if(this.targetList[p]==paths[pID].targets[0]){
					var newPath1 = getPathID(paths[pID].targets[0],T_c);
					var newPath2 = getPathID(paths[pID].targets[1],T_c);
				}else{
					var newPath1 = getPathID(paths[pID].targets[1],T_c);
					var newPath2 = getPathID(paths[pID].targets[0],T_c);
				}
				////print("Paths: "+newPathsInTour);
				//print("Targets: "+newTargetsInTour);
				if(paths[newPath1].isPermenent && paths[newPath2].isPermenent){
					dummyTour = this.clone();
					dummyTour.pathList.splice(p,1,newPath1,newPath2);// replace the element at index p with
					dummyTour.targetList.splice(p+1,0,T_c);
					dummyTour.computeMeanUncertainty(); // update the meanUncertainty and mean uncertainty gain
					//print("Targets: "+newTargetsInTour);
					var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
					var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
					//////print("gain:"+gain+", max: "+maxGain);
					if(gain>maxGain && dummyTour.meanUncertainty!=-1){
						maxGain	= gain;
						bestTour = dummyTour.clone();
			
					}
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


	this.addTheBestAvailableTargetAdvanced = function(){

		// target list which are not in the path
		var candidateTargets = [];
		for(var i = 0; i<this.allowedTargetList.length; i++){
			if(!this.targetList.includes(this.allowedTargetList[i])){
				candidateTargets.push(this.allowedTargetList[i]);
			}
		}
		print("Search space: T:"+candidateTargets);
		// end finding search space targets



		// method 1 to extend the cycle

		var maxGain1 = -1;
		var bestTour1 = -1;
		var dummyTour = this.clone();
		
		for(var i = 0; i<candidateTargets.length; i++){
			// for each candidate target, need to find the best path to remove from the existing tour
			// to maximize the gain of adding the candidate target;
			
			var T_i = candidateTargets[i];
			print("T_i:"+T_i);

			var meanUncertaintyIfNotCovered = targets[T_i].initialUncertainty + 0.5*targets[T_i].uncertaintyRate*periodT;

			for(var p = 0; p < this.pathList.length; p++){ // path to be removed so that T_c can be added via two new paths
				var pID = this.pathList[p];

				
				if(this.targetList[p]==paths[pID].targets[0]){
					var newPath1 = getPathID(paths[pID].targets[0],T_i);
					var newPath2 = getPathID(paths[pID].targets[1],T_i);
				}else{
					var newPath1 = getPathID(paths[pID].targets[1],T_i);
					var newPath2 = getPathID(paths[pID].targets[0],T_i);
				}
				////print("Paths: "+newPathsInTour);
				//print("Targets: "+newTargetsInTour);

				if(paths[newPath1].isPermenent && paths[newPath2].isPermenent){
					dummyTour = this.clone();
					dummyTour.pathList.splice(p,1,newPath1,newPath2);// replace the element at index p with
					dummyTour.targetList.splice(p+1,0,T_i);
					dummyTour.computeMeanUncertaintyAdvanced(); // update the meanUncertainty and mean uncertainty gain
					//print("Targets: "+newTargetsInTour);
					var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
					var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
					//print("diff:"+(gain-meanUncertaintyIfNotCovered)+"gain:"+gain+", max: "+maxGain1);
					if(gain>maxGain1 && dummyTour.meanUncertainty!=-1){
						maxGain1 = gain;
						bestTour1 = dummyTour.clone();
					}
				}	
				

			}

		}
		// end method 1



		// Method 2 to extend the cycle
		
		var maxGain2 = -1;
		var bestTour2 = -1;
		var dummyTour = this.clone();

		for(var i = 0; i<candidateTargets.length; i++){
			// for each candidate target, need to find the best way to connect to 
			// the current cycle
			
			var T_i = candidateTargets[i];
			print("T_i:"+T_i);

			var meanUncertaintyIfNotCovered = targets[T_i].initialUncertainty + 0.5*targets[T_i].uncertaintyRate*periodT;

			for(var j = 0; j < this.targetList.length; j++){ // path to be removed so that T_c can be added via two new paths
				var T_j = this.targetList[j]; // target T_j is to be added to the cycle

				////print("Paths: "+newPathsInTour);
				//print("Targets: "+newTargetsInTour);
				var newPath = getPathID(T_i,T_j);
				
				if(paths[newPath].isPermenent){
					dummyTour = this.clone();
					dummyTour.pathList.splice(j,0,newPath,newPath);
					dummyTour.targetList.splice(j+1,0,T_i,T_j);		
					dummyTour.computeMeanUncertaintyAdvanced(); // update the meanUncertainty and mean uncertainty gain

					//print("Targets: "+newTargetsInTour);
					var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
					var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
					//////print("gain:"+gain+", max: "+maxGain);
					if(gain>maxGain2 && dummyTour.meanUncertainty!=-1 ){
						maxGain2 = gain;
						bestTour2 = dummyTour.clone();
					}	
				}
				

			}

		}

		// end method 1



		// Method 3 to extend the cycle
		
		var maxGain3 = -1;
		var bestTour3 = -1;
		var dummyTour = this.clone();
		var L = this.targetList.length;
		for(var i = 0; i<candidateTargets.length; i++){
			// for each candidate target, need to find the best way to connect to 
			// the current cycle
			
			var T_i = candidateTargets[i];
			print("T_i:"+T_i);

			var meanUncertaintyIfNotCovered = targets[T_i].initialUncertainty + 0.5*targets[T_i].uncertaintyRate*periodT;

			// need to find two targets in the current cycle connected via set of aux targets
			// so that those aux targets can be eliminated from the cycle, without compromizing the cyclic shape

			for(var j = 0; j < this.targetList.length-2; j++){
				var T_j = this.targetList[j];

				for(var k = j+2; k<this.targetList.length; k++){
					var T_k = this.targetList[k];

					var newPath1 = getPathID(T_j,T_i);
					var newPath2 = getPathID(T_i,T_k);
					
					if(paths[newPath1].isPermenent && paths[newPath2].isPermenent){
						
						// search for targets inbetween T_j and T_k in targetlist
						var connectedViaAuxTargetsFwd = true;
						for(var ind = j + 1; ind < k; ind++){//fwd searcg
							if(!this.subCycleList[ind].includes(0)){
								connectedViaAuxTargetsFwd = false;
								break;// to break
							}
						}
						var connectedViaAuxTargetsBwd = true;
						for(var ind = j - 1; ind != k; ind--){//fwd searcg
							if(ind==-1){ind = this.targetList.length-1;}
							if(!this.subCycleList[ind].includes(0)){
								connectedViaAuxTargetsBwd = false;
								break;// to break
							}
						}

						dummyTour = this.clone();	
						if(connectedViaAuxTargetsFwd){
							dummyTour.pathList.splice(j,k-j,newPath1,newPath2);// replace the element at index p with
							dummyTour.targetList.splice(j+1,k-j-1,T_i);
							
							print("Method 3: Fwd j="+j+"; T_i="+T_i+"; k="+k);
							print("paths ji="+newPath1+"; ik="+newPath2);
							
							print("Targets B: "+this.targetList);
							print("Targets A: "+dummyTour.targetList);
							print("Paths B: "+this.pathList);
							print("Paths A: "+dummyTour.pathList);
						
						}else if(connectedViaAuxTargetsBwd){
							if(j>0){
								dummyTour.pathList.splice(0,j,newPath1);// replace the element at index p with
								dummyTour.pathList.push(newPath2);
								dummyTour.targetList.splice(0,j,T_i);	
							}else{
								dummyTour.pathList.splice(k,L-k,newPath2,newPath1);// replace the element at index p with
								dummyTour.targetList.splice(k+1,L-k-1,T_i);
							}
							
							print("Method 3: Bwd j="+j+"; T_i="+T_i+"; k="+k);
							print("paths ji="+newPath1+"; ik="+newPath2);
							print("Targets B: "+this.targetList);
							print("Targets A: "+dummyTour.targetList);
							print("Paths B: "+this.pathList);
							print("Paths A: "+dummyTour.pathList);

						}else{
							break; // to try different T_i, T_j combination
						}
						dummyTour.computeMeanUncertaintyAdvanced(); // update the meanUncertainty and mean uncertainty gain

						//print("Targets: "+newTargetsInTour);
						var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
						var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
						//////print("gain:"+gain+", max: "+maxGain);
						if(gain>maxGain3 && dummyTour.meanUncertainty!=-1){
							maxGain3 = gain;
							bestTour3 = dummyTour.clone();
						}	
					}

					
				}
			}

		}
		// end method  3



		if(bestTour1==-1 && bestTour2==-1 && bestTour3==-1){
			print("No way to improve!");
			print("Final Cycle: "+this.targetList+", cost: "+this.meanUncertainty.toFixed(2));
			return -1;
		}else if(maxGain1>=maxGain2 && maxGain1>=maxGain3){
			print("Improve via normal way!")
			print("Result: "+bestTour1.targetList);	
			this.cloneFrom(bestTour1);
			return +1;	
		}else if(maxGain2>=maxGain1 && maxGain2>=maxGain3){
			print("Improve via Aux Target!")
			print("Result: "+bestTour2.targetList);	
			this.cloneFrom(bestTour2);
			return +1;	
		}else{
			print("Improve via Aux Target Cancellation!")
			print("Result: "+bestTour3.targetList);	
			this.cloneFrom(bestTour3);
			return +1;
		}



	}


	this.computeMeanUncertainty = function(){

		// to compute the mean uncertainty gain of the cycle (i.e. meanuncertainty diffrence{notcovered-coverd})
		var meanUncertaintyIfNotCovered = 0;

		var B_A = [];
		var A = [];
		var dist = 0;
		var BMinusA	= [];

		for(var i = 0; i < this.targetList.length; i++){
			
			var T_i = this.targetList[i];
			B_A.push([]);// new row
			for(var j = 0; j<this.targetList.length; j++){
				if(i==j){
					B_A[i].push(agents[this.deployedAgent].sensingRate-targets[T_i].uncertaintyRate);
				}else{
					B_A[i].push(-targets[T_i].uncertaintyRate);
				}
			}
			
			A.push([targets[T_i].uncertaintyRate]);
			////print("error:")
			
			////print("pathList:"+this.pathList+", i="+i)
			dist = dist + paths[this.pathList[i]].distPath();
			BMinusA.push([agents[this.deployedAgent].sensingRate-targets[T_i].uncertaintyRate]);
		
			meanUncertaintyIfNotCovered = meanUncertaintyIfNotCovered + targets[T_i].initialUncertainty + 0.5*targets[T_i].uncertaintyRate*periodT;

		}
		//print("B_A="+B_A);
		var t_cy = dist/agents[this.deployedAgent].maxLinearVelocity;
		//print(t_cy);
		try{
			var dwellTimes = math.multiply(math.inv(B_A),math.multiply(A,t_cy));
		}
		catch(error){
			return -1;
		}

		
		var meanUncertainty = 0.5*math.multiply(math.transpose(BMinusA),dwellTimes);
		var meanUncertaintyGain = meanUncertaintyIfNotCovered - meanUncertainty;

		print("Targets: "+this.targetList+": "+meanUncertainty.toFixed(2));


		if(meanUncertainty<0 || meanUncertainty > 10000000){
			print("Error: det(B_A) = "+math.det(B_A));
			print(B_A);
			meanUncertainty = -1;
		}

		this.meanUncertainty = meanUncertainty;
		
	}



	this.computeMeanUncertaintyAdvanced = function(){// auxiliary targets are taken into account

		// to compute the mean uncertainty gain of the cycle (i.e. meanuncertainty diffrence{notcovered-coverd})
		var meanUncertaintyIfNotCovered = 0;


		// each target in the cycle should have an sub-cycle
		// computing sub-cycle parameters:
		var L = this.targetList.length;
		var travelTimeArray = [];
		var subCycleList = [];
		////print("targetList:"+this.targetList);
		for(var i = 0; i<L; i++){
			
			var T_i = this.targetList[i]; // target index in "targets" list
			var T_iprev; if(i==0){T_iprev = this.targetList[L-1];}else{T_iprev = this.targetList[i-1];}

			// traveltime
			travelTimeArray[i] = distP2(targets[T_i].position,targets[T_iprev].position)/agents[this.deployedAgent].maxLinearVelocity;

			// need to do a back search
			var subCycleArray = [...math.zeros(L)._data];
			var searchIndex = i;
			do{
				subCycleArray[searchIndex] = 1;
				searchIndex = searchIndex - 1; if(searchIndex==-1){searchIndex = L-1;}
			}while(T_i!=this.targetList[searchIndex])
			////print("i = "+i+"; subCycleArray = "+subCycleArray);	
			subCycleList.push(subCycleArray);

		}
		////print("Travel Times: "+travelTimeArray);
		// sub-cycle travel times:
		var subCycleTravelTimes = [];
		for(var i = 0; i<L; i++){
			subCycleTravelTimes.push(math.multiply(subCycleList[i],travelTimeArray));
		}
		////print("Sub-cycle travel times: "+subCycleTravelTimes);

		this.travelTimeList = [...travelTimeArray]; // travel time between adjadcent targets
		this.subCycleList = [...subCycleList]; // array of arrays: L x L : 1's and 0's 
		this.subCycleTravelTimeList = [...subCycleTravelTimes]; // M x 1 : t_ij values 

		// end computing sub-cycle parameters:


		// solving for dwell times
		// Construction of B_A_M and A_M matrices
		var B_A_M = [];
		var A_Mt = [];

		for(var i = 0; i < L; i++){
			
			var T_i = this.targetList[i];
			B_A_M.push([]);// new row
			for(var j = 0; j < L; j++){
				if(i==j){
					B_A_M[i].push(agents[this.deployedAgent].sensingRate-targets[T_i].uncertaintyRate);
				}else if(this.subCycleList[i][j]==0){
					B_A_M[i].push(0);
				}else{
					B_A_M[i].push(-targets[T_i].uncertaintyRate);
				}
			}
			A_Mt.push([targets[T_i].uncertaintyRate*this.subCycleTravelTimeList[i]]);
		}
		////print("B_A_M="+B_A_M);
		try{
			var dwellTimeList = math.multiply(math.inv(B_A_M),A_Mt);
		}
		catch(error){
			return -1;
		}
		var dwellTimeList = math.transpose(dwellTimeList);
		var dwellTimeList = dwellTimeList[0];

		////print("Dwell times="+dwellTimeList);
		

		// subcycle dwell times
		var subCycleDwellTimes = [];
		for(var i = 0; i<L; i++){
			subCycleDwellTimes.push(math.multiply(this.subCycleList[i],dwellTimeList));
		}
		////print("Sub-cycle dwell times: "+subCycleDwellTimes);

		this.dwellTimeList = [...dwellTimeList];
		this.subCycleDwellTimeList = [...subCycleDwellTimes]; 

		// end solving for dwell times



		// finding revised A and B parameters (on auxiliary targets)
		// we actually needs only the (B-A)_revised matrix
		var BMinusA	= [];
		var T_cyc = math.multiply(math.ones(L),math.add(this.travelTimeList,this.dwellTimeList));
		////print("T_cyc = "+T_cyc);
		for(var i = 0; i < L; i++){
			
			var T_i = this.targetList[i];
			var effectiveRatio = (this.subCycleTravelTimeList[i]+this.subCycleDwellTimeList[i])/T_cyc;
			BMinusA.push([effectiveRatio*(agents[this.deployedAgent].sensingRate-targets[T_i].uncertaintyRate)]);
		
		}
		////print("B-A="+BMinusA);
		// b-a found

		// computing cost
		var meanUncertainty = 0.5*math.multiply(math.transpose(BMinusA),this.dwellTimeList);

		print("Targets: "+this.targetList+": J="+meanUncertainty.toFixed(2));
		if(meanUncertainty<0 || meanUncertainty > 10000000){
			print("Error: det(B_A_M) = "+math.det(B_A_M));
			print(B_A_M);
			meanUncertainty = -1;
		}

		this.meanUncertainty = meanUncertainty;

		
	}


	this.swap2OPT = function(i,k){
		
		var dummyTour = new Cycle(this.deployedAgent,this.allowedTargetList);
		dummyTour.RGCComputingMode = this.RGCComputingMode;
		dummyTour.cycleRefiningParameters = [...this.cycleRefiningParameters];

		for(var p = 0; p<i; p++){
			dummyTour.pathList[p] = this.pathList[p];
			dummyTour.targetList[p] = this.targetList[p];
		}
		dummyTour.pathList[i] = getPathID(this.targetList[i],this.targetList[k]);

		dummyTour.targetList[i] = this.targetList[i];
		
		var count = 1;
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

		print("2OPT-Targets: "+dummyTour.targetList);
		//print("swapped Targets: "+dummyTour.targetList);

		dummyTour.computeMeanUncertainty();
		var cycleError = dummyTour.recorrectCycleErrors();

		var cost = dummyTour.meanUncertainty;
		
		dummyTour.highlight();
		if(!paths[dummyTour.pathList[i]].isPermenent || !paths[dummyTour.pathList[k]].isPermenent || cycleError==-1){cost=this.meanUncertainty+10;}
		
		if(cost<this.meanUncertainty){
			
			print("Paths: "+this.pathList);
			print("swapped Paths: "+dummyTour.pathList);
			
			print("Cost: "+this.meanUncertainty);
			print("Swapped Cost: "+cost);

			consolePrint("Steady state mean uncertainty reduced by "+(this.meanUncertainty-cost).toFixed(3)+" via 2-OPT method.");	
			this.cloneFrom(dummyTour);
			
		} 

		
		//print("Swapped cost: "+cost);
			
	}



	this.swap3OPT = function(i,j,k,l){

		// Type 1 - nominal - cost = this.meanUncertainty
		if(l==0){
			
			minCost = this.meanUncertainty;

			// Type 1
			var dummyTour = new Cycle(this.deployedAgent,this.allowedTargetList);
			dummyTour.RGCComputingMode = this.RGCComputingMode;
			dummyTour.cycleRefiningParameters = [...this.cycleRefiningParameters];

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
			print("swapped1 Targets: "+dummyTour.targetList.join());
			print("swapped1 Paths: "+dummyTour.pathList.join());

			
			
			dummyTour.computeMeanUncertainty();
			var cost = dummyTour.meanUncertainty;
			

			if(cost<minCost){
				minCost	= cost;
				print("Prev cost: "+this.meanUncertainty);
				print("Swapped1 Cost: "+cost);

				consolePrint("Steady state mean uncertainty reduced by "+(this.meanUncertainty-minCost).toFixed(3)+" via 3-OPT method.");
				print("Cost improved from 3-OPT!!!");

				this.cloneFrom(dummyTour);
				
			} 
			
			dummyTour.highlight();
		
		}else if(l==1){

			// Type 2
			
			var dummyTour = new Cycle(this.deployedAgent,this.allowedTargetList);
			dummyTour.RGCComputingMode = this.RGCComputingMode;
			dummyTour.cycleRefiningParameters = [...this.cycleRefiningParameters];

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
			
			var cost = dummyTour.meanUncertainty; 
			if(cost<minCost){
				minCost	= cost;
				print("Prev cost: "+this.meanUncertainty);
				print("Swapped2 Cost: "+cost);

				consolePrint("Steady state mean uncertainty reduced by "+(this.meanUncertainty-minCost).toFixed(3)+" via 3-OPT method.");
				print("Cost improved from 3-OPT!!!");

				this.cloneFrom(dummyTour);
				
			} 
			dummyTour.highlight();


		}else if(l==2){

			// Type 3
			
			var dummyTour = new Cycle(this.deployedAgent,this.allowedTargetList);
			dummyTour.RGCComputingMode = this.RGCComputingMode;
			dummyTour.cycleRefiningParameters = [...this.cycleRefiningParameters];

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
			
			var cost = dummyTour.meanUncertainty;
			if(cost<minCost){
				minCost	= cost;
				print("Prev cost: "+this.meanUncertainty);
				print("Swapped3 Cost: "+cost);

				consolePrint("Steady state mean uncertainty reduced by "+(this.meanUncertainty-minCost).toFixed(3)+" via 3-OPT method.");
				print("Cost improved from 3-OPT!!!");	

				this.cloneFrom(dummyTour);
				
			} 
			dummyTour.highlight();

		}
			
	}





	// threshold based on TSP
	this.computeThresholds = function(){

		var blockingThreshold = 1000;
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
	            		agents[z].threshold[p][q] = blockingThreshold;	
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
	            			agents[z].threshold[p][q] = blockingThreshold;		
	            		}
	            	}
	            } 
	        }
	    }

	    displayThresholds();
	}


	this.computeThresholdsAdvanced = function(){

		var blockingThreshold = 1000;


	    // biase the threshods
	    var agentID = this.deployedAgent;
	    if(cycles.length==0){
	    	consolePrint("Generate initial trajectory first (Use 'Search' button)!!!");
	    	return;
	    }
   
	    


	    // need to get last auxiliary target's total subcycle time
	    var L = this.targetList.length;
	    var M = targets.length;
	    var auxTargetData = [];
	    for(var i = 0; i<M; i++){
	    	auxTargetData.push([]);// if an element is empty, it means no other aux target
	    }
	    

	    for(var  i = 0; i<L; i++){
	    	var T_i = this.targetList[i];
			var searchIndex = i;
			do{
				searchIndex = searchIndex - 1; if(searchIndex==-1){searchIndex = L-1;}		
			}while(T_i!=this.targetList[searchIndex])
			
			var prevSearchIndex;
			if(searchIndex<L-1){prevSearchIndex = searchIndex+1;}else{prevSearchIndex = 0;}
			var nextIndex;
			if(i<L-1){nextIndex = i+1;}else{nextIndex = 0;}

			if(searchIndex!=i){//aux target exists!
				print(this.targetList);
				print("Aux target exist for i="+i+"; T_i="+T_i+"; lastind ="+prevSearchIndex+"; nextint"+nextIndex);
				auxTargetData[T_i].push([prevSearchIndex,i,nextIndex]); // store the last aux target index in the target list
			}else{
				print(this.targetList);
				print("No aux targets for i="+i+"; T_i="+T_i);
			}
	    }
	    print(auxTargetData);




	    var pathList = this.pathList;
	    var targetList = this.targetList;


	    if(this.travelTimeList.length == 0){
	    	print("Computing missing data for agent "+(this.deployedAgent+1));
	    	this.computeMeanUncertaintyAdvanced();
	    }

	    var T_cyc = math.multiply(math.ones(L),math.add(this.travelTimeList,this.dwellTimeList));
	    print("T_cyc="+T_cyc);
	   	var z = agentID;
	    for(var p = 0; p < targets.length; p++){
	        for(var q = 0; q < targets.length; q++){
	            
	            if(p==q){// policy
	            	agents[z].threshold[p][q] = 0;
	            }else{
	            	var path_pq = getPathID(p,q);
	            	if(!pathList.includes(path_pq) && paths[path_pq].isPermenent){
	            		agents[z].threshold[p][q] = blockingThreshold;	
	            	}else{
	            		print("p"+p+"; q"+q);
	            		if(auxTargetData[p].length!=0){
	            			// new method
	            			print("Aug data exists!")
	            			var valueApplied = false;
	            			for(k = 0; k<auxTargetData[p].length; k++){
	            				var pastIndex = auxTargetData[p][k][0];
	            				var currentIndex = auxTargetData[p][k][1];
	            				var nextIndex = auxTargetData[p][k][2];
	            				if(p==this.targetList[currentIndex] && q==this.targetList[nextIndex]){
	            					print("Decide policy: T_"+(p+1)+" to T_"+(q+1)+"; carefully!");
	            					//  note that T_j_n = q = this.targetList[auxTargetData[p][k][1]]; 
	            					var upperBound = targets[this.targetList[nextIndex]].uncertaintyRate*(T_cyc - this.dwellTimeList[nextIndex] - this.travelTimeList[nextIndex]);
	            					print("upper: "+upperBound+"; to goto T_"+(this.targetList[nextIndex]+1));
	            					
	            					var lowerBound = targets[this.targetList[nextIndex]].uncertaintyRate*(T_cyc - this.subCycleDwellTimeList[currentIndex] - this.subCycleTravelTimeList[currentIndex] - this.dwellTimeList[nextIndex] - this.travelTimeList[nextIndex]);
	            					print("lower: "+lowerBound+"; to block T_"+(this.targetList[pastIndex]+1));
	            					var r = 0.5;
	            					var base = lowerBound+r*(upperBound-lowerBound);
	            					
	            					if(!thresholdGenerationMethod){base = 0;}
	            					agents[z].threshold[p][q] = base;
	            					valueApplied = true;
	            				}
	            			}
	            			if(!valueApplied){
	            				agents[z].threshold[p][q] = blockingThreshold;
	            			}
	    
	            	
	            		}else{ // old method

	            			// path_pq is in path list, but can be repeated
	            			// need to find whether agent goeas trom target p to q in cycle
	            			var agentHasGoneFrompToq = false;
	            			for(var k = 0; k<targetList.length; k++){
	            				var presentTarget = targetList[k]; 
	            				var nextTarget;
	            				if(k==targetList.length-1){
	            					nextTarget = targetList[0];
	            				}else{
	            					nextTarget = targetList[k+1];
	            				}
	            				if(presentTarget == p && nextTarget == q){
	            					print("Agent has gone from: T_"+(p+1)+" to T_"+(q+1)+" directly!");
	            					agentHasGoneFrompToq = true;
	            				}
	            			}

		            		if(agentHasGoneFrompToq){
		            			agents[z].threshold[p][q] = 0;		
		            		}else if(!paths[path_pq].isPermenent){
		            			agents[z].threshold[p][q] = 10000;	
		            		}else{
		            			agents[z].threshold[p][q] = blockingThreshold;		
		            		}
		            	}
	            	}
	            } 
	        }
	    }

	    displayThresholds();
	}

	this.recorrectCycleErrors = function(){
		
		if(this.targetList.length == 1){
			return;
		}


		for(var i = 0; i<this.targetList.length-1; i++){
			var T_i = this.targetList[i];
			var T_j = this.targetList[i+1];

			var pID = getPathID(T_i,T_j);
			if(this.pathList[i]!=pID){
				print("Path corrected at index:"+i+", from:");
				print(this.pathList);
				this.pathList[i] = pID;
				print("to:");
				print(this.pathList);
			}
			if(T_i==T_j){
				print("Path error, repeating targets!");
				print(this.targetList);
				return -1;
			}
		}
		var T_i = this.targetList[i];
		var T_j = this.targetList[0];

		var pID = getPathID(T_i,T_j);
		if(this.pathList[i]!=pID){
			print("Path corrected at index:"+i+", from:");
			print(this.pathList);
			this.pathList[i] = pID;
			print("to:");
			print(this.pathList);
		}
		if(T_i==T_j){
			print("Path error, repeating targets!");
			print(this.targetList);
			return -1;
		}else{
			return +1;
		}
	
	}

	this.assignAgentToCycle = function(){
		var targetID = this.targetList[0];
		var agentID = this.deployedAgent;
		agents[agentID].position = targets[targetID].position;
        agents[agentID].residingTarget = [targetID];
        agents[agentID].initialResidingTarget = targetID;
                
	}

	this.removeUnwantedAuxiliaryTargets = function(j,k){
		// var maxGain3 = -1;
		// var bestTour3 = -1;
		// var dummyTour = this.clone();
		var L = this.targetList.length;

		var T_j = this.targetList[j];
		var T_k = this.targetList[k];

		var newPath = getPathID(T_j,T_k);
		
		if(paths[newPath].isPermenent && !this.pathList.includes(newPath)){
			
			// search for targets inbetween T_j and T_k in targetlist
			var connectedViaAuxTargetsFwd = true;
			for(var ind = j + 1; ind < k; ind++){//fwd searcg
				if(!this.subCycleList[ind].includes(0)){
					connectedViaAuxTargetsFwd = false;
					break;// to break
				}
			}
			var connectedViaAuxTargetsBwd = true;
			for(var ind = j - 1; ind != k; ind--){//fwd searcg
				if(ind==-1){ind = this.targetList.length-1;}
				if(!this.subCycleList[ind].includes(0)){
					connectedViaAuxTargetsBwd = false;
					break;// to break
				}
			}

			dummyTour = this.clone();	
			if(connectedViaAuxTargetsFwd){
				dummyTour.pathList.splice(j,k-j,newPath);// replace the element at index p with
				dummyTour.targetList.splice(j+1,k-j,T_k);// same
				
				print("Method 3: Fwd j="+j+"; k="+k);
				print("paths jk="+newPath);
				
				print("Targets B: "+this.targetList);
				print("Targets A: "+dummyTour.targetList);
				print("Paths B: "+this.pathList);
				print("Paths A: "+dummyTour.pathList);
			
			}else if(connectedViaAuxTargetsBwd){
				if(j>0){
					dummyTour.pathList.splice(0,j,newPath);// replace the element at index p with
					dummyTour.targetList.splice(0,j+1,T_j);	
				}else{
					dummyTour.pathList.splice(k,L-k,newPath);// replace the element at index p with
					dummyTour.targetList.splice(k+1,L-k,T_k);
				}
				
				print("Method 3: Bwd j="+j+"; k="+k);
				print("paths jk="+newPath);
				print("Targets B: "+this.targetList);
				print("Targets A: "+dummyTour.targetList);
				print("Paths B: "+this.pathList);
				print("Paths A: "+dummyTour.pathList);

			}else{
				return;
			}
			dummyTour.computeMeanUncertaintyAdvanced(); // update the meanUncertainty and mean uncertainty gain

			//print("Targets: "+newTargetsInTour);
			var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
			var gain = this.meanUncertainty - meanUncertaintyOfTheTour;
			//////print("gain:"+gain+", max: "+maxGain);
			if(gain > 0 && dummyTour.meanUncertainty!=-1){
				consolePrint("Aux target removing can lead to gain: "+gain);
				this.cloneFrom(dummyTour);
			}
			dummyTour.highlight();	
		}else{
			return;
		}

	}




	this.addTargetInTheBestWay = function(T_i){

		if(this.targetList.includes(T_i)){
			print("Target T_"+(T_i+1)+" is already included in the cycle!");
			return -1;
		}

		var meanUncertaintyIfNotCovered = targets[T_i].initialUncertainty + 0.5*targets[T_i].uncertaintyRate*periodT;


		// method 1 to extend the cycle

		var maxGain1 = -Infinity;
		var bestTour1 = -1;
		var dummyTour = this.clone();
		
		
		// for the candidate target, need to find the best path to remove from the existing tour
		// to maximize the gain of adding the candidate target;
		

		for(var p = 0; p < this.pathList.length; p++){ // path to be removed so that T_c can be added via two new paths
			var pID = this.pathList[p];

			
			if(this.targetList[p]==paths[pID].targets[0]){
				var newPath1 = getPathID(paths[pID].targets[0],T_i);
				var newPath2 = getPathID(paths[pID].targets[1],T_i);
			}else{
				var newPath1 = getPathID(paths[pID].targets[1],T_i);
				var newPath2 = getPathID(paths[pID].targets[0],T_i);
			}
			////print("Paths: "+newPathsInTour);
			//print("Targets: "+newTargetsInTour);

			if(paths[newPath1].isPermenent && paths[newPath2].isPermenent){
				dummyTour = this.clone();
				dummyTour.pathList.splice(p,1,newPath1,newPath2);// replace the element at index p with
				dummyTour.targetList.splice(p+1,0,T_i);
				dummyTour.computeMeanUncertaintyAdvanced(); // update the meanUncertainty and mean uncertainty gain
				//print("Targets: "+newTargetsInTour);
				var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
				var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
				//////print("gain:"+gain+", max: "+maxGain);
				if(gain>maxGain1 && dummyTour.meanUncertainty!=-1){
					maxGain1 = gain;
					bestTour1 = dummyTour.clone();
				}
			}			

		}

		
		// end method 1



		// Method 2 to extend the cycle
		
		var maxGain2 = -Infinity;
		var bestTour2 = -1;
		var dummyTour = this.clone();

		
		// for each candidate target, need to find the best way to connect to 
		// the current cycle

		for(var j = 0; j < this.targetList.length; j++){ // path to be removed so that T_c can be added via two new paths
			var T_j = this.targetList[j]; // target T_j is to be added to the cycle

			////print("Paths: "+newPathsInTour);
			//print("Targets: "+newTargetsInTour);
			var newPath = getPathID(T_i,T_j);
			
			if(paths[newPath].isPermenent){
				dummyTour = this.clone();
				dummyTour.pathList.splice(j,0,newPath,newPath);
				if(dummyTour.pathList.length>2){
					dummyTour.targetList.splice(j+1,0,T_i,T_j);		
				}else{
					dummyTour.targetList.splice(j,1,T_i,T_j);
				}
				dummyTour.computeMeanUncertaintyAdvanced(); // update the meanUncertainty and mean uncertainty gain

				//print("Targets: "+newTargetsInTour);
				var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
				var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
				//////print("gain:"+gain+", max: "+maxGain);
				if(gain>maxGain2 && dummyTour.meanUncertainty!=-1 ){
					maxGain2 = gain;
					bestTour2 = dummyTour.clone();
				}	
			}
			

		}

		

		// end method 1



		// Method 3 to extend the cycle
		
		var maxGain3 = -Infinity;
		var bestTour3 = -1;
		var dummyTour = this.clone();
		var L = this.targetList.length;
		

		// for each candidate target, need to find the best way to connect to 
		// the current cycle
		
		
		// need to find two targets in the current cycle connected via set of aux targets
		// so that those aux targets can be eliminated from the cycle, without compromizing the cyclic shape

		for(var j = 0; j < this.targetList.length-2; j++){
			var T_j = this.targetList[j];

			for(var k = j+2; k<this.targetList.length; k++){
				var T_k = this.targetList[k];

				var newPath1 = getPathID(T_j,T_i);
				var newPath2 = getPathID(T_i,T_k);
				
				if(paths[newPath1].isPermenent && paths[newPath2].isPermenent){
					
					// search for targets inbetween T_j and T_k in targetlist
					var connectedViaAuxTargetsFwd = true;
					for(var ind = j + 1; ind < k; ind++){//fwd searcg
						if(!this.subCycleList[ind].includes(0)){
							connectedViaAuxTargetsFwd = false;
							break;// to break
						}
					}
					var connectedViaAuxTargetsBwd = true;
					for(var ind = j - 1; ind != k; ind--){//fwd searcg
						if(ind==-1){ind = this.targetList.length-1;}
						if(!this.subCycleList[ind].includes(0)){
							connectedViaAuxTargetsBwd = false;
							break;// to break
						}
					}

					dummyTour = this.clone();	
					if(connectedViaAuxTargetsFwd){
						dummyTour.pathList.splice(j,k-j,newPath1,newPath2);// replace the element at index p with
						dummyTour.targetList.splice(j+1,k-j-1,T_i);
						
						print("Method 3: Fwd j="+j+"; T_i="+T_i+"; k="+k);
						print("paths ji="+newPath1+"; ik="+newPath2);
						
						print("Targets B: "+this.targetList);
						print("Targets A: "+dummyTour.targetList);
						print("Paths B: "+this.pathList);
						print("Paths A: "+dummyTour.pathList);
					
					}else if(connectedViaAuxTargetsBwd){
						if(j>0){
							dummyTour.pathList.splice(0,j,newPath1);// replace the element at index p with
							dummyTour.pathList.push(newPath2);
							dummyTour.targetList.splice(0,j,T_i);	
						}else{
							dummyTour.pathList.splice(k,L-k,newPath2,newPath1);// replace the element at index p with
							dummyTour.targetList.splice(k+1,L-k-1,T_i);
						}
						
						print("Method 3: Bwd j="+j+"; T_i="+T_i+"; k="+k);
						print("paths ji="+newPath1+"; ik="+newPath2);
						print("Targets B: "+this.targetList);
						print("Targets A: "+dummyTour.targetList);
						print("Paths B: "+this.pathList);
						print("Paths A: "+dummyTour.pathList);

					}else{
						break; // to try different T_i, T_j combination
					}
					dummyTour.computeMeanUncertaintyAdvanced(); // update the meanUncertainty and mean uncertainty gain

					//print("Targets: "+newTargetsInTour);
					var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
					var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
					//////print("gain:"+gain+", max: "+maxGain);
					if(gain>maxGain3 && dummyTour.meanUncertainty!=-1){
						maxGain3 = gain;
						bestTour3 = dummyTour.clone();
					}	
				}

				
			}
		}

		
		// end method  3



		if(bestTour1==-1 && bestTour2==-1 && bestTour3==-1){
			print("No way to improve! - Error!!!");
			return -1;
		}else if(maxGain1>=maxGain2 && maxGain1>=maxGain3){
			print("Improve via normal way!")
			print("Result: "+bestTour1.targetList);	
			this.cloneFrom(bestTour1);
			return +1;	
		}else if(maxGain2>=maxGain1 && maxGain2>=maxGain3){
			print("Improve via Aux Target!")
			print("Result: "+bestTour2.targetList);	
			this.cloneFrom(bestTour2);
			return +1;	
		}else{
			print("Improve via Aux Target Cancellation!")
			print("Result: "+bestTour3.targetList);	
			this.cloneFrom(bestTour3);
			return +1;
		}



	}



	this.computeNeglectedTargetCost = function(){
		
		var neglectedTargetCost = 0;
		var neglectedTargetListString = "(";

		for(var i = 0; i<this.allowedTargetList.length; i++){
			
			var T_i = this.allowedTargetList[i];
			if(!this.targetList.includes(T_i)){
				neglectedTargetCost = neglectedTargetCost + targets[i].initialUncertainty + 0.5*targets[i].uncertaintyRate*periodT;
				neglectedTargetListString = neglectedTargetListString+(i+1)+",";
			}

		}
		neglectedTargetListString = neglectedTargetListString.slice(0,-1) + ")";
		return([neglectedTargetCost,neglectedTargetListString]);
	
	}


	this.iterationOfComputingGreedyCycles = function(delay){

		print("Cycle length of agent "+this.deployedAgent+" is "+this.targetList.length+" targets.");
		
		if(this.RGCComputingMode==1){//initial greedy
			
			sleepFor(delay);
			var val = this.addTheBestAvailableTarget(); // cannot add more to this cycle
			if(val==-1){

				var neglectedTargetInfo = this.computeNeglectedTargetCost();
				if(neglectedTargetInfo[0] > 0){
					consolePrint("Unvisited targets assigned to agent "+(this.deployedAgent+1)+": "+neglectedTargetInfo[1]+" contributes to the objective J by "+neglectedTargetInfo[0].toFixed(3)+".");
				}else{
					consolePrint("All targets assigned to agent "+(this.deployedAgent+1)+" are covered.");
				}
				var meanSystemUncertaintyVal = this.meanUncertainty + neglectedTargetInfo[0];
				consolePrint("Steady state mean cycle uncertainty of agent "+(this.deployedAgent+1)+" (J_"+(this.deployedAgent+1)+") ="+meanSystemUncertaintyVal.toFixed(3)+", achieved via greedy cycle search (Method 1).");

				this.recorrectCycleErrors();
				if(RGCComputingMode!=3){this.assignAgentToCycle();}

				if(this.pathList.length>3){
					this.RGCComputingMode = 2;
					this.cycleRefiningParameters[0] = [0,2]; //[i,k] for 2-opt
				}else{
					this.RGCComputingMode = 0;
				}
				sleepFor(delay);
			}
			
		}else if(this.RGCComputingMode==2){//2-opt
			var i = this.cycleRefiningParameters[0][0];
			var k = this.cycleRefiningParameters[0][1];
			sleepFor(10);
			this.swap2OPT(i,k);
			
			k = k + 1;
			if(k >= this.pathList.length){
				i = i + 1;
				if(i >= this.pathList.length-2){
					consolePrint("2-Opt refining stage for agent "+(this.deployedAgent+1)+" finished!");
					this.RGCComputingMode = 3;
					this.cycleRefiningParameters[1] = [0,1,2,0]; // i,j,k,l for 3-Opt
					sleepFor(delay);
				}
				k = i + 2;
			}
			
			this.cycleRefiningParameters[0] = [i,k];
		
		}else if(this.RGCComputingMode==3){//3-opt

			var i = this.cycleRefiningParameters[1][0];
			var j = this.cycleRefiningParameters[1][1];
			var k = this.cycleRefiningParameters[1][2];
			var l = this.cycleRefiningParameters[1][3];
			////sleepFor(10);
			this.swap3OPT(i,j,k,l);
			
			l = l + 1;
			if(l >= 3){	
				k = k + 1;
				if(k >= this.pathList.length){
					j = j + 1;
					if(j >= this.pathList.length-1){
						i = i + 1;
						if(i >= this.pathList.length-2){
							consolePrint("3-Opt refining stage for agent "+(this.deployedAgent+1)+" finished!");
							this.RGCComputingMode = 0;							

							var neglectedTargetInfo = this.computeNeglectedTargetCost();
							if(neglectedTargetInfo[0] > 0){
								consolePrint("Unvisited targets assigned to agent "+(this.deployedAgent+1)+": "+neglectedTargetInfo[1]+" contributes to the objective J by "+neglectedTargetInfo[0].toFixed(3)+".");
							}else{
								consolePrint("All targets assigned to agent "+(this.deployedAgent+1)+" are covered.");
							}
							var meanSystemUncertaintyVal = this.meanUncertainty + neglectedTargetInfo[0];
							consolePrint("Steady state mean cycle uncertainty of agent "+(this.deployedAgent+1)+" (J_"+(this.deployedAgent+1)+") ="+meanSystemUncertaintyVal.toFixed(3)+", achieved after refining (RGC-Method 1).");
							////sleepFor(delay);
						}
						j = i + 1;
					}
					k = j + 1;
				}
				l = 0;
			}
			this.cycleRefiningParameters[1] = [i,j,k,l];

		}
	}


	this.iterationOfComputingGreedyCyclesAdvanced = function(delay){

		print("Cycle length of agent "+this.deployedAgent+" is "+this.targetList.length+" targets.");

		if(this.RGCComputingMode==1){//initial greedy
			sleepFor(delay);

			var val = this.addTheBestAvailableTargetAdvanced();
			
			if(val==-1){
				
				var neglectedTargetInfo = this.computeNeglectedTargetCost();
				if(neglectedTargetInfo[0] > 0){
					consolePrint("Unvisited targets assigned to agent "+(this.deployedAgent+1)+": "+neglectedTargetInfo[1]+" contributes to the objective J by "+neglectedTargetInfo[0].toFixed(3)+".");
				}else{
					consolePrint("All targets assigned to agent "+(this.deployedAgent+1)+" are covered.");
				}
				var meanSystemUncertaintyVal = this.meanUncertainty + neglectedTargetInfo[0];
				consolePrint("Steady state mean cycle uncertainty of agent "+(this.deployedAgent+1)+" (J_"+(this.deployedAgent+1)+") ="+meanSystemUncertaintyVal.toFixed(3)+", achieved via greedy cycle search (Method 1).");

				this.recorrectCycleErrors();
				if(RGCComputingMode!=3){this.assignAgentToCycle();}
				

				if(this.pathList.length>3){
					this.RGCComputingMode = 2;
					this.cycleRefiningParameters[0] = [0,2]; //[i,k] for 2-opt
				}else{
					this.RGCComputingMode = 0;
				}
				sleepFor(delay);
			}
			
		}else if(this.RGCComputingMode==2){//2-opt ---> not actually
			var j = this.cycleRefiningParameters[0][0];
			var k = this.cycleRefiningParameters[0][1];
			sleepFor(10);
			this.removeUnwantedAuxiliaryTargets(j,k);
			
			k = k + 1;
			if(k >= this.targetList.length){
				j = j + 1;
				if(j >= this.targetList.length-2){
					consolePrint("Aux target removing stage finished of agent "+(this.deployedAgent+1)+"!");
					this.RGCComputingMode = 0; // no 3-opt yet
					this.cycleRefiningParameters[1] = [0,1,2,0]; // i,j,k,l for 3-Opt
					sleepFor(delay);
				}
				k = j + 2;
			}
			this.cycleRefiningParameters[0] = [j,k];
		
		}else if(this.RGCComputingMode==3){//3-opt

			var i = this.cycleRefiningParameters[1][0];
			var j = this.cycleRefiningParameters[1][1];
			var k = this.cycleRefiningParameters[1][2];
			var l = this.cycleRefiningParameters[1][3];
			sleepFor(10);
			this.swap3OPT(i,j,k,l);
			
			l = l + 1;
			if(l >= 3){	
				k = k + 1;
				if(k >= this.pathList.length){
					j = j + 1;
					if(j >= this.pathList.length-1){
						i = i + 1;
						if(i >= this.pathList.length-2){
							consolePrint("3-Opt refining stage finished!");
							this.RGCComputingMode = 0;
							sleepFor(delay);
						}
						j = i + 1;
					}
					k = j + 1;
				}
				l = 0;
			}
			this.cycleRefiningParameters[1] = [i,j,k,l];

		}
	}



	this.computeNeglectedTargetCostDirect = function(){
		var meanUncertaintyOfNeglected = 0;
        for(var i = 0; i<this.allowedTargetList.length; i++){
        	var T_j = this.allowedTargetList[i];
        	if(!this.targetList.includes(T_j)){
        		meanUncertaintyOfNeglected = meanUncertaintyOfNeglected + targets[T_j].initialUncertainty + 0.5*targets[T_j].uncertaintyRate*periodT;
        	}
        }
        return meanUncertaintyOfNeglected;
	}



	//// stuff for the bargaining

	this.effectOfAbandoningInternalTarget = function(internalTarget){
		
		var T_i = internalTarget;    
        var meanUncertaintyIfNotCovered = targets[T_i].initialUncertainty + 0.5*targets[T_i].uncertaintyRate*periodT;


		// need to compute the greedy cycle (refined) only considering the allowed target set - {internalTarget}
		if(!this.targetList.includes(internalTarget) && this.allowedTargetList.includes(internalTarget)){// already abandoned!
			
			print("Already abandoned target. It would be a help if someone can taee care of this!");
			var dummyTour = this.clone();
			var ind1 = dummyTour.allowedTargetList.indexOf(internalTarget);
			var allowedList = [...dummyTour.allowedTargetList];
			allowedList.splice(ind1,1);
			dummyTour.allowedTargetList = [...allowedList]; // remove the inernalTarget
			
        	var costOfAbandoning = this.meanUncertainty+this.computeNeglectedTargetCostDirect()-(dummyTour.meanUncertainty+dummyTour.computeNeglectedTargetCostDirect()+meanUncertaintyIfNotCovered);
        	print("Cost of abandoning null target: "+costOfAbandoning);
			return [1,costOfAbandoning,dummyTour];
		
		}else if(!this.targetList.includes(internalTarget) && !this.allowedTargetList.includes(internalTarget)){// already abandoned!
			print("Not a relevent target. No effect of loosing it!");
			return [-1,0,this];
		}else if(this.targetList.length==2){ // surgically have to remove the one agent out of two
			if(this.targetList[0]==internalTarget){
				var T_o = this.targetList[1]; 
			}else{
				var T_o = this.targetList[0]; 
			}
			var dummyTour = this.clone();
			dummyTour.pathList = [];
			dummyTour.targetList = [T_o];
			var gain = this.meanUncertainty+this.computeNeglectedTargetCostDirect()-(meanUncertaintyIfNotCovered)			
			return [1,gain,dummyTour]; // no-way to abandon (lets take 2 target cycles to be our base line)
		}else if(this.targetList.length<2){
			return [-1,0,this]; // no-way to abandon
		}


		var dummyTour = this.clone();

		var ind1 = dummyTour.allowedTargetList.indexOf(internalTarget);
		var allowedList = [...dummyTour.allowedTargetList];
		allowedList.splice(ind1,1);
		dummyTour.allowedTargetList = [...allowedList]; // remove the inernalTarget

		// initial greedy step
		dummyTour.computeBestInitialCycle();
		dummyTour.RGCComputingMode = 1; 

		print("Inner greedy search to remove T_"+(internalTarget+1));
		var iterationCount = 1;
		while(dummyTour.RGCComputingMode!=0){ 
			print("Inner greedy iteration "+iterationCount);
			if(!cycleGenerationMethod){
	            dummyTour.iterationOfComputingGreedyCycles(0);  
	        }else{
	            dummyTour.iterationOfComputingGreedyCyclesAdvanced(0);  
	        }
	        iterationCount = iterationCount+1;
        }

        print("Constructed new tour: ");
        print(dummyTour.targetList);   	
    	
    	// var meanUncertaintyOfNeglected = 0;
     //    for(var i = 0; i<dummyTour.allowedTargetList.length; i++){
     //    	var T_j = dummyTour.allowedTargetList[i];
     //    	if(!dummyTour.targetList.includes(T_j)){
     //    		meanUncertaintyOfNeglected = meanUncertaintyOfNeglected + targets[T_j].initialUncertainty + 0.5*targets[T_j].uncertaintyRate*periodT;
     //    	}
     //    }

        var costOfAbandoning = this.meanUncertainty+this.computeNeglectedTargetCostDirect()-(dummyTour.meanUncertainty+dummyTour.computeNeglectedTargetCostDirect()+meanUncertaintyIfNotCovered);
        print("Cost of abandoning: "+costOfAbandoning);

        return [+1,costOfAbandoning,dummyTour];

	}






	this.effectOfInsertionOfExternalTargetAdvanced = function(externalTargetIndex){

		// this function returns: [validity, gain, requiredCycle]
		// validity : -1 if there is no physical way to connect the given external target
		// otherwise it is +1
		// gain : if the external target is a neglected one, what would be the gain of adding it (gain can be negative!)
		// requiredCycle : gives the best extended cycle which contains the external target 

		var T_i = externalTargetIndex;
		var meanUncertaintyIfNotCovered = targets[T_i].initialUncertainty + 0.5*targets[T_i].uncertaintyRate*periodT;


		// for the single isolated agent case
		if(this.targetList.length==1){
			var dummyTour = this.clone();
			var p = getPathID(this.targetList[0],T_i);
			if(paths[p].isPermenent){
				dummyTour.pathList = [p,p];
			    dummyTour.targetList = [dummyTour.targetList[0],T_i];
			    dummyTour.computeMeanUncertaintyAdvanced();

			    var gain = this.meanUncertainty + meanUncertaintyIfNotCovered - dummyTour.meanUncertainty;
				print("Improve via direct addition!")
				print("Result: "+dummyTour.targetList);	
				return [+1, gain, dummyTour];
			}else{
				print("No way to append the given external target to the isolated single target cycle");
				return [-1,0,this];
			}
		}else if(this.targetList.includes(T_i)){
			print("The external target is already included!")
			return [-1,0,this];
		}
		// end single isolated agent case







		// method 1 to extend the cycle

		var maxGain1 = -Infinity;
		var bestTour1 = -1;
		var dummyTour = -1;
		// for each candidate target, need to find the best path to remove from the existing tour
		// to maximize the gain of adding the candidate target;

		for(var p = 0; p < this.pathList.length; p++){ // path to be removed so that T_c can be added via two new paths
			var pID = this.pathList[p];

			
			if(this.targetList[p]==paths[pID].targets[0]){
				var newPath1 = getPathID(paths[pID].targets[0],T_i);
				var newPath2 = getPathID(paths[pID].targets[1],T_i);
			}else{
				var newPath1 = getPathID(paths[pID].targets[1],T_i);
				var newPath2 = getPathID(paths[pID].targets[0],T_i);
			}
			////print("Paths: "+newPathsInTour);
			//print("Targets: "+newTargetsInTour);

			if(paths[newPath1].isPermenent && paths[newPath2].isPermenent){
				dummyTour = this.clone();
				dummyTour.pathList.splice(p,1,newPath1,newPath2);// replace the element at index p with
				dummyTour.targetList.splice(p+1,0,T_i);
				dummyTour.computeMeanUncertaintyAdvanced(); // update the meanUncertainty and mean uncertainty gain
				//print("Targets: "+newTargetsInTour);
				var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
				var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
				//print("diff:"+(gain-meanUncertaintyIfNotCovered)+"gain:"+gain+", max: "+maxGain1);
				if(gain>maxGain1 && dummyTour.meanUncertainty!=-1){
					maxGain1 = gain;
					bestTour1 = dummyTour.clone();
				}
			}	
			

		}
		// end method 1


		// Method 2 to extend the cycle
		
		var maxGain2 = -Infinity;
		var bestTour2 = -1;
		for(var j = 0; j < this.targetList.length; j++){ // path to be removed so that T_c can be added via two new paths
			var T_j = this.targetList[j]; // target T_j is to be added to the cycle

			////print("Paths: "+newPathsInTour);
			//print("Targets: "+newTargetsInTour);
			var newPath = getPathID(T_i,T_j);
			
			if(paths[newPath].isPermenent){
				dummyTour = this.clone();
				dummyTour.pathList.splice(j,0,newPath,newPath);
				dummyTour.targetList.splice(j+1,0,T_i,T_j);		
				dummyTour.computeMeanUncertaintyAdvanced(); // update the meanUncertainty and mean uncertainty gain

				//print("Targets: "+newTargetsInTour);
				var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
				var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
				//////print("gain:"+gain+", max: "+maxGain);
				if(gain>maxGain2 && dummyTour.meanUncertainty!=-1 ){
					maxGain2 = gain;
					bestTour2 = dummyTour.clone();
				}	
			}
			
		}
		// end method 2



		// Method 3 to extend the cycle
		
		var maxGain3 = -Infinity;
		var bestTour3 = -1;
		var dummyTour = this.clone();
		var L = this.targetList.length;
		
		// need to find two targets in the current cycle connected via set of aux targets
		// so that those aux targets can be eliminated from the cycle, without compromizing the cyclic shape

		for(var j = 0; j < this.targetList.length-2; j++){
			var T_j = this.targetList[j];

			for(var k = j+2; k<this.targetList.length; k++){
				var T_k = this.targetList[k];

				var newPath1 = getPathID(T_j,T_i);
				var newPath2 = getPathID(T_i,T_k);
				
				if(paths[newPath1].isPermenent && paths[newPath2].isPermenent){
					
					// search for targets inbetween T_j and T_k in targetlist
					var connectedViaAuxTargetsFwd = true;
					for(var ind = j + 1; ind < k; ind++){//fwd searcg
						if(!this.subCycleList[ind].includes(0)){
							connectedViaAuxTargetsFwd = false;
							break;// to break
						}
					}
					var connectedViaAuxTargetsBwd = true;
					for(var ind = j - 1; ind != k; ind--){//fwd searcg
						if(ind==-1){ind = this.targetList.length-1;}
						if(!this.subCycleList[ind].includes(0)){
							connectedViaAuxTargetsBwd = false;
							break;// to break
						}
					}

					dummyTour = this.clone();	
					if(connectedViaAuxTargetsFwd){
						dummyTour.pathList.splice(j,k-j,newPath1,newPath2);// replace the element at index p with
						dummyTour.targetList.splice(j+1,k-j-1,T_i);
						
						print("Method 3: Fwd j="+j+"; T_i="+T_i+"; k="+k);
						print("paths ji="+newPath1+"; ik="+newPath2);
						
						print("Targets B: "+this.targetList);
						print("Targets A: "+dummyTour.targetList);
						print("Paths B: "+this.pathList);
						print("Paths A: "+dummyTour.pathList);
					
					}else if(connectedViaAuxTargetsBwd){
						if(j>0){
							dummyTour.pathList.splice(0,j,newPath1);// replace the element at index p with
							dummyTour.pathList.push(newPath2);
							dummyTour.targetList.splice(0,j,T_i);	
						}else{
							dummyTour.pathList.splice(k,L-k,newPath2,newPath1);// replace the element at index p with
							dummyTour.targetList.splice(k+1,L-k-1,T_i);
						}
						
						print("Method 3: Bwd j="+j+"; T_i="+T_i+"; k="+k);
						print("paths ji="+newPath1+"; ik="+newPath2);
						print("Targets B: "+this.targetList);
						print("Targets A: "+dummyTour.targetList);
						print("Paths B: "+this.pathList);
						print("Paths A: "+dummyTour.pathList);

					}else{
						break; // to try different T_i, T_j combination
					}
					dummyTour.computeMeanUncertaintyAdvanced(); // update the meanUncertainty and mean uncertainty gain

					//print("Targets: "+newTargetsInTour);
					var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
					var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
					//////print("gain:"+gain+", max: "+maxGain);
					if(gain>maxGain3 && dummyTour.meanUncertainty!=-1){
						maxGain3 = gain;
						bestTour3 = dummyTour.clone();
					}	
				}
				
			}
		}
		
		// end method  3

		if(bestTour1==-1 && bestTour2==-1 && bestTour3==-1){
			print("No way to improve!");
			print("Final Cycle: "+this.targetList+", cost: "+this.meanUncertainty.toFixed(2));
			return [-1,-1,this];
		}else if(maxGain1>=maxGain2 && maxGain1>=maxGain3){
			print("Improve via normal way!")
			print("Result: "+bestTour1.targetList);	
			////this.cloneFrom(bestTour1);
			return [+1,maxGain1,bestTour1];	
		}else if(maxGain2>=maxGain1 && maxGain2>=maxGain3){
			print("Improve via Aux Target!")
			print("Result: "+bestTour2.targetList);	
			////this.cloneFrom(bestTour2);
			return [+1,maxGain2,bestTour2];	
		}else{
			print("Improve via Aux Target Cancellation!")
			print("Result: "+bestTour3.targetList);	
			////this.cloneFrom(bestTour3);
			return [+1,maxGain3,bestTour3];
		}

	}



	this.effectOfInsertionOfExternalTarget = function(externalTargetIndex){

		// this function returns: [validity, gain, requiredCycle]
		// validity : -1 if there is no physical way to connect the given external target
		// otherwise it is +1
		// gain : if the external target is a neglected one, what would be the gain of adding it (gain can be negative!)
		// requiredCycle : gives the best extended cycle which contains the external target 


		var maxGain = -1;
		var minMeanUncertainty = -1;
		var bestTour = -1;
		
		
		// for each candidate target, need to find the best path to remove from the existing tour
		// to maximize the gain of adding the candidate target;
		
		var T_c =  externalTargetIndex;
		var meanUncertaintyIfNotCovered = targets[T_c].initialUncertainty + 0.5*targets[T_c].uncertaintyRate*periodT;



		// for the single isolated agent case
		if(this.targetList.length==1){

			var dummyTour = this.clone();
			var p = getPathID(dummyTour.targetList[0],T_c);
			if(paths[p].isPermenent){
				dummyTour.pathList = [p,p];
			    dummyTour.targetList = [dummyTour.targetList[0],T_c];
			    dummyTour.computeMeanUncertaintyAdvanced();

			    var gain = this.meanUncertainty + meanUncertaintyIfNotCovered - dummyTour.meanUncertainty;
				print("Improve via direct addition!")
				print("Result: "+dummyTour.targetList);	
				return [+1, gain, dummyTour];
			}else{
				print("No way to append the given external target to the isolated single target cycle");
				return [-1,0,this];
			}

		}else if(this.targetList.includes(T_c)){
			print("The external target is already included!")
			return [-1,0,this];
		}
		// end single isolated agent case




		for(var p = 0; p < this.pathList.length; p++){ // path to be removed so that T_c can be added via two new paths
			var pID = this.pathList[p];

			
			if(this.targetList[p]==paths[pID].targets[0]){
				var newPath1 = getPathID(paths[pID].targets[0],T_c);
				var newPath2 = getPathID(paths[pID].targets[1],T_c);
			}else{
				var newPath1 = getPathID(paths[pID].targets[1],T_c);
				var newPath2 = getPathID(paths[pID].targets[0],T_c);
			}
			////print("Paths: "+newPathsInTour);
			//print("Targets: "+newTargetsInTour);
			if(paths[newPath1].isPermenent && paths[newPath2].isPermenent){
				dummyTour = this.clone();
				dummyTour.pathList.splice(p,1,newPath1,newPath2);// replace the element at index p with
				dummyTour.targetList.splice(p+1,0,T_c);
				dummyTour.computeMeanUncertainty(); // update the meanUncertainty and mean uncertainty gain
				//print("Targets: "+newTargetsInTour);
				var meanUncertaintyOfTheTour = dummyTour.meanUncertainty;
				var gain = this.meanUncertainty  + meanUncertaintyIfNotCovered - meanUncertaintyOfTheTour;
				//////print("gain:"+gain+", max: "+maxGain);
				if(gain>maxGain && dummyTour.meanUncertainty!=-1){
					maxGain	= gain;
					bestTour = dummyTour.clone();
		
				}
			}	
			

		}

		
		if(bestTour==-1){
			print("No way to improve!");
			print("Final Cycle: "+this.targetList+", cost: "+this.meanUncertainty.toFixed(2));
			return [-1,-1,this];
		}else{
			print("Result: "+bestTour.targetList);	
			////this.cloneFrom(bestTour);
			return [+1,maxGain,bestTour];		
		}


	}






}




function executeABargainingStep(){

	// generate offeres
	var offersList = [];
	for(var i = 0; i<cycles.length; i++){
		for(var j = 0; j<targets.length; j++){
			var offer;
			if(!cycleGenerationMethod){
	            offer = cycles[i].effectOfInsertionOfExternalTarget(j);  
	        }else{
	            offer = cycles[i].effectOfInsertionOfExternalTargetAdvanced(j);  
	        }

	        if(offer[0]==1){
	        	offer.splice(1,0,i,j);
	        	offersList.push(offer);
	        }
	    }
	}
	if(offersList.length>0){
		print("Number of offers found: "+offersList.length);
		print(offersList)
	}else{
		print("No offers found!");
		RGCComputingMode = 4;
		return;
	}

	// end generating offers

	
	// need to pick the best - independedt set of offers
	var bestDealValue = -Infinity;
	var bestDeal = -1;
	for(var i = 0; i<cycles.length; i++){// each agent/cycle look for the relevent best offer to them from each other agents
		for(var k = 0; k<offersList.length; k++){
			
			var T_i = offersList[k][2]
			if(cycles[i].targetList.includes(offersList[k][2])){
				// agent j has made us (i) an offer to take the target T_i
				var dummyTour = cycles[i].clone();
				var rebuttle = dummyTour.effectOfAbandoningInternalTarget(T_i);
				if(rebuttle[1]+offersList[k][3] > bestDealValue && rebuttle[0]!=-1){
					bestDealValue = rebuttle[1]+offersList[k][3];
					rebuttle.splice(1,0,i); // who has to act (i.e. to contract accepting the offer)
					bestDeal = [offersList[k],rebuttle];
				}
			}

		}
			
	}
	print("Net effect of best offer: "+bestDealValue);
	print(bestDeal)
	// end picking a offer
	

	// execute best deal
	if(bestDealValue>0){
		
		
		var A_i = bestDeal[0][1]; // best offer is made by agent A_i 
		cycles[A_i].cloneFrom(bestDeal[0][4]);
		
		var T_added = bestDeal[0][2]; // the target added to A_i
		var allowedList = [...bestDeal[0][4].allowedTargetList];
		allowedList.push(T_added);
		cycles[A_i].allowedTargetList = [...allowedList]; // remove the inernalTarget


		var A_j = bestDeal[1][1]; // contraction has to be done to agent A_j
		cycles[A_j].cloneFrom(bestDeal[1][3]);
		
		print("New cycles");
		print(cycles[A_i]);

		print(cycles[A_j]);
		updateTargetClustersUsingCycles();
		consolePrint("A Clustering (and cycle) adjustment found with gain "+bestDealValue.toFixed(3)+".")

	}else{
		
		RGCComputingMode = 4;
		var systemUncertainty = 0;
		for(var i = 0; i<cycles.length; i++){
			systemUncertainty = systemUncertainty + cycles[i].meanUncertainty;
		}	
		consolePrint("No further benificial clustering (and cycle) adjustments were found.")
		consolePrint("Final Theoretical Mean System Uncertainty: "+systemUncertainty.toFixed(3)+".")
	}
	// end executing best deal

	



}

function initiateInterCycleTargetBargaining(){
	
	if(agents.length>1 && targetClusters.length==0 && cycles.length==0){
		consolePrint("Cluster the targets first! (Using the 'Cluster !' button.)");
		return;
	}else if(agents.length==1 && cycles.length==0){// the usual single agent case
		consolePrint("Generate the agent cycle first! (Using the 'Search !' button.), then, no need to use this function.");
		return;
	}else if(RGCComputingMode < 2){
		consolePrint("Generate the agent cycles first! (Using the 'Search !' button.), then, use this function.");
		return;
	}else{
		consolePrint("Start adjustig initial cycles!");
		RGCComputingMode = 3;
		return;
	}
	// else RGCComputingMode >= 2 ---> we need to  adjust ! 

}




function initiateComputingRefinedGreedyCycles(){
	
	if(agents.length>1 && targetClusters.length==0){
		consolePrint("Cluster the targets first! (Using the 'Cluster !' button.)")
		return;
	}else if(agents.length==1 && cycles.length==0){// the usual single agent case
		applyFoundClustersToGroupTargets(math.zeros(targets.length)._data);
		// var agentID	= 0;
		// var targetSet = math.range(0,targets.length)._data;
		// cycles.push(new Cycle(agentID,targetSet)); //creating a cycle for the agent (with agent id) on the target set
	}

	for(var j = 0; j<agents.length; j++){
		cycles[j].computeBestInitialCycle();
		cycles[j].RGCComputingMode = 1;  //start iteratively evolving the cycles
	}

	RGCComputingMode = 1; // global variable
	
}




function evaluateMeanSystemUncertainty(){
	var a = computeNeglectedTargetCost();
	var neglectedTargetCost = a[0];
	var meanSystemUncertaintyVal = 0;
	for(var j = 0; j<cycles.length;j++){
		meanSystemUncertaintyVal = meanSystemUncertaintyVal + cycles[j].meanUncertainty;
	}
	return meanSystemUncertaintyVal + neglectedTargetCost;
}


function computeNeglectedTargetCost(){
	var neglectedTargetCost = 0;
	var neglectedTargetListString = "(";

	for(var i = 0; i<targets.length; i++){
		var targetCovered = false;
		for(var j = 0; j<cycles.length; j++){
			if(cycles[j].targetList.includes(i)){
				targetCovered = true;
				break; // to exit
			}
		}
		if(!targetCovered){
			neglectedTargetCost = neglectedTargetCost + targets[i].initialUncertainty + 0.5*targets[i].uncertaintyRate*periodT;
			neglectedTargetListString = neglectedTargetListString+(i+1)+",";
		}

	}
	neglectedTargetListString = neglectedTargetListString.slice(0,-1) + ")";
	return([neglectedTargetCost,neglectedTargetListString]);
}



function resetCycles(){
	RGCComputingMode = 0;
	for(var i = cycles.length; i > 0; i--){
        removeACycle();
    }
}

function restartCycles(){
	RGCComputingMode = 0;
	for(var i = cycles.length; i > 0; i--){
        removeACycle();
    }
    if(targetClusters.length>0){
		var N = targetClusters.length; // number of agents
		for(var j = 0; j < N; j++){
	        cycles.push(new Cycle(j,targetClusters[j]));// deployedAgent,targetSet
	        agents[j].assignToTheTarget(targetClusters[j][0]);
	    }
	}
}


function generateThresholdsFromRoutes(){
	for(var i = 0; i<cycles.length; i++){
		if(cycleGenerationMethod){
			cycles[i].computeThresholdsAdvanced();
		}else{
			cycles[i].computeThresholds();
		}
	}
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

