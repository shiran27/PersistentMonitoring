
function sq(val){
    return Math.pow(val,2);
}

function isFullyConnected(){
    
    // lets check whether every other target is reachable from target 0
    var ans = findMinimumDistancesFrom(0);
    if(ans[0].includes(Infinity)){
        return false    
    }else{
        return true
    }
    
}


function findMinimumDistancesFrom(startTarget){

    var distances = [];
    var unvisitedTargets = [];
    var visitedTargets = [];
    var predecessors = [];

    for(var i = 0; i<targets.length; i++){
        
        if(i==startTarget){
            distances[i] = 0;
        }else{
            distances[i] = Infinity;
        }

        unvisitedTargets[i] = i;
        predecessors[i] = [];

    }


    while(visitedTargets.length<targets.length){
        
        // searching for minimum target with a minimum distance
        var minDistanceTarget;          //i corresponds to : d(i) = min(d(j); j \in unvisited)
        var minDistance = Infinity;     //d(i) corresponds to : d(i) = min(d(j); j \in unvisited)
        for(var j = 0; j<unvisitedTargets.length; j++){
            var T_j = unvisitedTargets[j];
            if(distances[T_j]<minDistance){
                minDistance = distances[T_j];
                minDistanceTarget = T_j;
            } 
        }

        // adding and removing the found target with minimum distance value
        visitedTargets.push(minDistanceTarget);
        unvisitedTargets.splice(unvisitedTargets.indexOf(minDistanceTarget),1);


        // searching the neighbors of minDistanceTarget and updating their distances if necessary
        for(var j = 0; j<targets[minDistanceTarget].neighbors.length; j++){
            var T_j = targets[minDistanceTarget].neighbors[j];
            if(T_j != minDistanceTarget){
                var pathID = getPathID(minDistanceTarget,T_j);
                var c_ij = paths[pathID].distPath(); // cost of visiting node T_j through the minDistanceTarget    
                var newDistanceToNeighbor = distances[minDistanceTarget]+c_ij;
                if(distances[T_j]>newDistanceToNeighbor){
                    distances[T_j] = newDistanceToNeighbor;
                    var newPathToNeighbor = [...predecessors[minDistanceTarget]];
                    newPathToNeighbor.push(minDistanceTarget);
                    predecessors[T_j] = [...newPathToNeighbor];
                }
            }

        }

    }


    // closing remarks
    for(var i = 0; i<targets.length; i++){
        predecessors[i].push(i);
    }

    return [distances,predecessors];


}




function findMinimumMeanCycleUncertaintiesFrom(startTarget){

    // dijkstras in the sense of persistent monitoring
    // lets first do it with the metric d_ij = distance between i and j
    // then lets replace it with more complex cycle tours
    
    var minimumMeanCycleUncertainties = []; // the cycle containing both startTarget and T_i  
    var unvisitedTargets = [];
    var visitedTargets = [];
    var predecessors = []; // basically cycles

    var agentID = 0;
    var targetSet = math.range(0,targets.length)._data;
    var nullCycle = new Cycle(agentID,targetSet);
    nullCycle.pathList = [];
    nullCycle.targetList = [startTarget];
    nullCycle.meanUncertainty = 0;

    for(var i = 0; i<targets.length; i++){
        
        if(i==startTarget){
            minimumMeanCycleUncertainties[i] = 0; // cycle containing only the startNode
        }else{
            minimumMeanCycleUncertainties[i] = Infinity;
        }

        unvisitedTargets[i] = i;
        predecessors[i] = nullCycle.clone();
    }


    while(visitedTargets.length<targets.length){
        
        // searching for the target with a minimum distance
        var minimumMeanCycleUncertaintyTarget;          //i corresponds to : d(i) = min(d(j); j \in unvisited)
        var minimumMeanCycleUncertainty = Infinity;     //d(i) corresponds to : d(i) = min(d(j); j \in unvisited)
        for(var j = 0; j<unvisitedTargets.length; j++){
            var T_j = unvisitedTargets[j];
            if(minimumMeanCycleUncertainties[T_j] < minimumMeanCycleUncertainty){
                minimumMeanCycleUncertainty = minimumMeanCycleUncertainties[T_j];
                minimumMeanCycleUncertaintyTarget = T_j; // T_i
            } 
        }
        var T_i = minimumMeanCycleUncertaintyTarget;
        var d_i = minimumMeanCycleUncertainty; // same as  minimumMeanCycleUncertainties[T_i]

        // adding and removing the found target with minimum distance value
        visitedTargets.push(T_i);
        unvisitedTargets.splice(unvisitedTargets.indexOf(T_i),1);


        // searching the neighbors of minDistanceTarget and updating their distances if necessary
        for(var j = 0; j<targets[T_i].neighbors.length; j++){
            var T_j = targets[T_i].neighbors[j];
            
            if(T_j != T_i){

                // var pathID = getPathID(T_i,T_j);
                // var c_ij = paths[pathID].distPath(); // cost of visiting node T_j through the minDistanceTarget    
                // var newDistanceToNeighbor = distances[minDistanceTarget]+c_ij;
                
                // if(distances[T_j]>newDistanceToNeighbor){
                //     distances[T_j] = newDistanceToNeighbor;
                //     var newPathToNeighbor = [...predecessors[minDistanceTarget]];
                //     newPathToNeighbor.push(minDistanceTarget);
                //     predecessors[T_j] = [...newPathToNeighbor];
                // }

                var dummyCycle = predecessors[T_i].clone(); // cycle containing startTarget and T_i
                dummyCycle.addTargetInTheBestWay(T_j);
                var newMeanCycleUncertaintyWithNeighbor = dummyCycle.meanUncertainty;

               
                if(minimumMeanCycleUncertainties[T_j] > newMeanCycleUncertaintyWithNeighbor){
                    minimumMeanCycleUncertainties[T_j] = newMeanCycleUncertaintyWithNeighbor;
                    predecessors[T_j].cloneFrom(dummyCycle);
                }
            }

        }

    }


    return [minimumMeanCycleUncertainties,predecessors];

}




function computeSimilarityMatrix(){
    
    var affinityMatrix = [];
    for(var i = 0; i<targets.length; i++){
        var minimumDistancesFound;
        if(similarityMeasureType==0){
            minimumDistancesFound = findMinimumDistancesFrom(i);
        }else if(similarityMeasureType==1){
            minimumDistancesFound = findMinimumMeanCycleUncertaintiesFrom(i);    
        }
        affinityMatrix[i] = [...minimumDistancesFound[0]]; // distances matrix from T_i;
    }

    return affinityMatrix;

}


function computeSimilarityGraph(){// using the similarity graph and neighborhood cluster width
    // similarity graph = weighted adjacency matrix
    
    var similarityMatrix  = computeSimilarityMatrix();
    
    var similarityGraph = []; // weighted adjacency matrix
    var degreeMatrix= []; 


    var M = targets.length;
    var sigma = neighborhoodWidthForClustering; // from the slider
    
    for(var i = 0; i < M; i++){
        
        similarityGraph.push([]);

        var rowSum = 0;
        degreeMatrix.push([]);
        
        for(var j = 0; j < M; j++){

            var w_ij = Math.exp(-0.5*sq(similarityMatrix[i][j]/sigma));
            // if(w_ij<0.5){
            //     w_ij = 0;    
            // }
            similarityGraph[i][j] = w_ij; 
            print("i = "+i+", j = "+j+", sim_ij = "+similarityMatrix[i][j]+"; w_ij = "+similarityGraph[i][j]);

            rowSum = rowSum + similarityGraph[i][j];
            degreeMatrix[i][j] = 0;
        }
        degreeMatrix[i][i] = rowSum;
    }

    return [similarityGraph,degreeMatrix];


}


function computeClusters(){
    
    var similarityMeasures  = computeSimilarityGraph();
    var W = [...similarityMeasures[0]]; //weightedAdjacencyMatrix
    var D = [...similarityMeasures[1]]; //DegreeMatrix
    

    var N = agents.length;

    // laplacian
    var L = math.add(D,math.multiply(W,-1));
    print("Base Laplacian: ");
    print(L);

    if(spectralClusteringMethod==1){    
        L = math.multiply(math.inv(D),L);
    }else if(spectralClusteringMethod==2){
        var d = math.diag(D);//diagonal entries
        var d_sqrt = math.sqrt(d);
        var D_psqrt = math.diag(d_sqrt);
        var D_nsqrt = math.inv(D_psqrt);
        L = math.multiply(math.multiply(D_nsqrt,L),D_psqrt);
    }else{
        // no need to change L
    }
    print("Transformed Laplacian: ");
    print(L);

   
    
    var eigDecomp = numeric.eig(L);
    print("Eigen decomp: ");
    print(eigDecomp); // eigenvectors are in columns, according to the order of the eigenvalues are listed
    
    if(typeof eigDecomp.E.y !== 'undefined'){
        consolePrint('Error in clustering !. See Laplacian matrix.');
        print('Error in clustering !. See Laplacian matrix.');
        return;
    } 


    // construct new data points in V \in \R^(MxN), selecting M rows of length N 
    // (N = number of agents = number of clusters = number of eigenvectors picked) 
    
    var V = eigDecomp.E.x;
    var lambda = eigDecomp.lambda.x;

    var dataPoints = extractDataPointsFrom(V,lambda);
    var clusteredPoints = kMeansCluster(dataPoints);
    print(clusteredPoints);

    
    applyFoundClustersToGroupTargets(clusteredPoints); // adjust parameters related to targets and paths and simulation
    

}


function extractDataPointsFrom(V,lambda){
    
    var N = agents.length;
    var M = targets.length;
    
    var sortedEigenvalues = [...lambda];

    sortedEigenvalues.sort(function(a, b){return a - b});
    print("Un-sorted eigenvalues:"+lambda);
    print("Sorted eigenvalues:"+sortedEigenvalues);


    // need to pick only the first non zero minimum amplitude eigenvalues!!!
    // var indexArray = [];
    // for(var i = 0; i < N; i++){
    //     var highestEigIndex = lambda.indexOf(sortedEigenvalues[i]);
    //     indexArray.push(highestEigIndex);
    // }

    var indexArray = [];
    for(var i = 1; i < N; i++){// neglecting the first eigenvalue which is 0 
        var highestEigIndex = lambda.indexOf(sortedEigenvalues[i]);
        indexArray.push(highestEigIndex);
    }


    var dataPoints = [];
    for(var i = 0; i < M; i++){
        dataPoints.push([]);
        for(var j = 0; j < indexArray.length; j++){
            var ind = indexArray[j];
            dataPoints[i][j] = V[i][ind];
        }
    }

    print("Selected Data Points:")
    print(dataPoints);

    if(spectralClusteringMethod==2){// datapoints matrix rows should be normalized 
        for(var i = 0; i < M; i++){// each row
            var rowSqSum = 0;
            for(var j = 0; j < N; j++){
                rowSqSum = rowSqSum + sq(dataPoints[i][j]);
            }

            for(var j = 0; j < N; j++){
                dataPoints[i][j] = dataPoints[i][j]/Math.sqrt(rowSqSum);
            }
            
        }
    }

    return dataPoints;
    
}



function kMeansCluster(dataPoints){

    var M = targets.length; // number of datapoints = M = number of targets = number of rows in dataPoints matrix
    var N = agents.length; // dimention of a datapoint = N = number of agents = number of columns in dataPoints matrix
    

    var minCostAssignment;
    var minAssignmentCost = Infinity;
    
    for(var k = 0; k<numberOfKMeansIterations; k++){
        // we need to compute N -clusters and the datapoints belonging to those clusters
        var means = []; // NxN
        
        // initial means
        for(var i = 0; i < N; i++){
            var ind = Math.floor(M*Math.random()); // randomly picked index
            means[i] = [...dataPoints[ind]];
        }
        print("Initial means:");
        print(means.join());
        

        // iterative update of means
        
        var isConverged = false;
        var iterationsCount = 0;
        var assignedCluster = [];
        var assignmentCost = 0;

        while(!isConverged){

            //Assignment
            assignedCluster = []; // 1xM; to what cluster that each data point (row in dataPoints matrix) belongs to
            assignmentCost = 0;

            for(var i = 0; i < M; i++){// for all the datapoints

                var assignedMeanIndex;
                var distanceToAssignedMean = Infinity;

                for(var j = 0; j < N; j++){// for all the means
                    var euclideonDistance = sq(math.norm(math.add(means[j],math.multiply(dataPoints[i],-1)))); 
                    if(euclideonDistance < distanceToAssignedMean){
                        assignedMeanIndex = j;
                        distanceToAssignedMean = euclideonDistance;
                    }
                    
                }
                //print("datapoint i="+i+" assinged to j="+assignedMeanIndex)
                assignedCluster[i] = assignedMeanIndex; 
                assignmentCost = assignmentCost + distanceToAssignedMean;
            }
            // end assignment


            // means update
            var convergedMeanCount = 0;
            
            for(var j = 0; j < N; j++){// computing each data center(i.e each mean)

                var sumDataPoint;
                var dataCount = 0;

                for(var i = 0; i < M; i++){ // for each data apoint

                    if(assignedCluster[i]==j){
                        if(dataCount==0){
                            sumDataPoint = [...dataPoints[i]];    
                        }else{
                            sumDataPoint = math.add(sumDataPoint,dataPoints[i]);
                        }
                        dataCount = dataCount + 1;    
                    }   
                }

                var newMean = math.multiply(sumDataPoint,1/dataCount);
                var distanceToNewMeanComponent = math.norm(math.add(newMean,math.multiply(means[j],-1)));
                //print("distance: "+distanceToNewMeanComponent+" for component j="+j+" in iter="+iterationsCount);
                if(distanceToNewMeanComponent==0){// mean component converged
                    convergedMeanCount = convergedMeanCount + 1;
                }

                means[j] = [...newMean];

            
            }
            print(means.join());

            // end means update


            // convergence cheack
            if(convergedMeanCount==N){
                isConverged = true;
                print("K-Means step in spectral clustering converged in "+iterationsCount+" iterations!.");
            }else if(iterationsCount > 200){
                isConverged = true;
                print("K-Means step in spectral clustering - not converged!.");
                assignmentCost = Infinity;
            }else{
                iterationsCount = iterationsCount + 1;
            }

        }

        print("Result in kmeans iteration "+k);
        print(assignedCluster);

        if(assignmentCost<minAssignmentCost){
            print("New min found!!!");
            minAssignmentCost = assignmentCost;
            minCostAssignment = assignedCluster;
        }
    }

    print("Minimum assignment cost observed: " + minAssignmentCost);
    return minCostAssignment;
    

}


function applyFoundClustersToGroupTargets(clusteredPoints){ // adjust parameters related to targets and paths and simulation
    
    var N = agents.length;
    var M = targets.length;
    var P = paths.length;
    
    resetGraphClusters(); // this might be redundent but its safer

    // creating structure
    targetClusters = [];
    for(var j = 0; j < N; j++){
        targetClusters[j] = [];
    }

    // creating target clusters
    for(var i = 0; i < M; i++){
        var ind = clusteredPoints[i]
        targetClusters[ind].push(i);    
    }


    // identifying inter cluster paths - for disp;ay  purposes
    interClusterPaths = [];    
    for(var p = 0; p < P; p++){// check all paths
        paths[p].brokenDueToClustering = false;

        for(var j = 0; j < N; j++){ // check over all target clusters
            var L1 = targetClusters[j].includes(paths[p].targets[0]);
            var L2 = targetClusters[j].includes(paths[p].targets[1]);
            if(L1 ^ L2){
            //if( ((!L1) && L2) || (L1 && (!L2)) ){
                interClusterPaths.push(p);
                paths[p].brokenDueToClustering = true;
            } 
        }
    }



    // creating cycles
    resetCycles(); // to make cycles = [];
    for(var j = 0; j < N; j++){
        cycles.push(new Cycle(j,targetClusters[j]));// deployedAgend,targetSet
        //////agents[j].assignToTheTarget(targetClusters[j][0]);
    }

    displayClustersMode = true;

}

function projectToPositiveAxis(num){
    if(num<0){
        return 0;
    }else{
        return num;
    }
}



function updateTargetClustersUsingCycles(){

    var N = agents.length;
    var M = targets.length;
    var P = paths.length;
    
    // creating structure
    targetClusters = [];
    for(var j = 0; j < N; j++){
        targetClusters[j] = [];
    }

    // creating target clusters
    for(var i = 0; i < N; i++){
        targetClusters[i] = [...cycles[i].targetList];    
    }


    // identifying inter cluster paths - for disp;ay  purposes
    interClusterPaths = [];    
    for(var p = 0; p < P; p++){// check all paths
        paths[p].brokenDueToClustering = false;

        for(var j = 0; j < N; j++){ // check over all target clusters
            var L1 = targetClusters[j].includes(paths[p].targets[0]);
            var L2 = targetClusters[j].includes(paths[p].targets[1]);
            if( ((!L1) && L2) || (L1 && (!L2)) ){
                interClusterPaths.push(p);
                paths[p].brokenDueToClustering = true;
            } 
        }
    }

    for(var j = 0; j < N; j++){
        //////agents[j].assignToTheTarget(targetClusters[j][0]);
    }

    displayClustersMode = true;

}

function plusOneToArray(values){// for indexes displaying purposes
    var result = [];
    for(var j = 0; j<values.length; j++){
        result.push(values[j]+1);
    }
    return result;
}




function solveRootsOfAQuadratic(A,B,C){
    var delta = sq(B) - 4*A*C
    if(delta<0){// imag root
        return [false,NaN,NaN];
    }else if(A==0){//1 real non-rep root
        var root1 = -C/B;
        if(root1<=0){
            return [false,root1,NaN];
        }else{
            return [true,root1,NaN];
        }
        return []
    }else{// two roots
        var root1 = (-B+Math.sqrt(delta))/(2*A);
        var root2 = (-B-Math.sqrt(delta))/(2*A);
        if((C/A)>=0 && (-B/A)<0){// both negative roots 
            return [false,root1,root2];
        }else{// atleast one is positive
            return [true,max(root1,root2),min(root1,root2)];
        }
    } 
}



// From : https://javascriptsource.com/quartic-equation-solver/
function calcmult(a2,b2,c2,d2,e2) {
  var real = a2*c2 - b2*d2
  var img = b2*c2 + a2*d2

  if (e2 == 0) {
    return real
  } else {
    return img
  }
}

function isquareroot(a1,b1,n1) {
  var y = Math.sqrt((a1*a1) + (b1*b1));
  var y1 = Math.sqrt((y - a1) / 2);
  var x1 = b1 / (2*y1);

  if (n1 == 0) {
    return x1
  } else {
    return y1
  }
}


function solveRootsOfAQuartic(aq,bq,cq,dq,eq) {
  

  // Extract X^4 Coefficent
  var aq2 = aq // Keeps Orignial AQ value
  // Extract X^3 Coefficent
  var bq2 = bq // Keeps Orignial BQ Value
  // Extract X^2 Coefficent
  // Extract X Coefficent
  // Extract Constant
  // Define Perfect Quartic Varible
  var perfect = 0;
  var perfectbiquadratic = 0;

  // The Bi-Quadratic 2 Perfect Squares that are negative test
  if (cq*cq - 4*aq*eq == 0 && cq > 0) {
    perfectbiquadratic = 1;
  }

  // Divide Equation by the X^4 Coefficent to make equation in the form of X^4 + AX^3 + BX^2 + CX + D
  bq /= aq;
  cq /= aq;
  dq /= aq;
  eq /= aq;
  aq = 1;
  var f2 = cq - (3*bq*bq / 8);
  var g2 = dq + (bq*bq*bq/8) - (bq*cq/2);
  var h2 = eq - (3*bq*bq*bq*bq/256) + (bq*bq*(cq/16)) - (bq*dq/4);
  var a = 1;
  var b = f2/2
  var c = (f2*f2 - (4*h2)) / 16
  var d = -1*((g2*g2)/64)

  if (b == 0 && c == 0 && d == 0) {
    perfect = 1
  }

  // Cubic routine starts here…..
  var f = (((3*c) / a) - ((b*b) / (a*a))) / 3;
  var g = (((2*b*b*b) / (a*a*a)) - ((9*b*c) / (a*a)) + ((27*d) / a)) / 27
  var h = eval(((g*g)/4) + ((f*f*f)/27))
  var z = 1/3;
  var i;
  var j;
  var k;
  var l;
  var m;
  var n;
  var p;
  var xoneterm;
  var xtwoterm;
  var xthreeterm;
  var alreadydone;
  var alreadydone2 = 0;
  var ipart = 0;
  var p = 0
  var q = 0
  var r = 0
  var s = 0

  if (h <= 0) {
    var exec = 2
    i = Math.sqrt(((g*g) / 4) - h);
    j = Math.pow(i,z);
    k = Math.acos(-1 * (g / (2*i)));
    l = -1*j;
    m = Math.cos(k / 3);
    n = Math.sqrt(3) * Math.sin(k / 3);
    p = (b / (3*a)) * -1;
    xoneterm = (2*j) * Math.cos(k/3) - (b / (3*a));
    xtwoterm = l * (m + n) + p;
    xthreeterm = l * (m - n) + p;
  }

  if (h > 0) {
    var exec = 1
    var R = (-1*(g / 2)) + Math.sqrt(h);
    if (R < 0) {
      var S = -1*(Math.pow((-1*R),z))
    } else {
      var S = Math.pow(R,z);
    }
    var T = (-1*(g / 2)) - Math.sqrt(h);
    if (T < 0) {
      var U = -1*(Math.pow((-1*T),z));
    } else {
    var U = Math.pow(T,z);
    }

    xoneterm = (S + U) - (b / (3*a));
    xtwoterm = (-1*(S+U)/2) - (b / (3*a));
    var ipart = ((S-U) * Math.sqrt(3)) / 2;
    xthreeterm = xtwoterm;
  }

  if (f == 0 && g == 0 && h == 0) {
    if ((d/a) < 0 ) {
      xoneterm = (Math.pow((-1*(d/a)),z));
      xtwoterm = xoneterm;
      xthreeterm = xoneterm;
    } else {
      xoneterm = -1*(Math.pow((d/a),z));
      xtwoterm = xoneterm;
      xthreeterm = xoneterm;
    }
  }
  // ….and ends here.

  // Return to solving the Quartic.
  if (ipart == 0 && xoneterm.toFixed(10) == 0) {
    var alreadydone2 = 1
    var p2 = Math.sqrt(xtwoterm)
    var q = Math.sqrt(xthreeterm)
    var r = -g2 / (8*p2*q)
    var s = bq2/(4*aq2)
  }

  if (ipart == 0 && xtwoterm.toFixed(10) == 0 && alreadydone2 == 0 && alreadydone2 != 1) {
    var alreadydone2 = 2
    var p2 = Math.sqrt(xoneterm)
    var q = Math.sqrt(xthreeterm)
    var r = -g2 / (8*p2*q)
    var s = bq2/(4*aq2)
  }

  if (ipart == 0 && xthreeterm.toFixed(10) == 0 && alreadydone2 == 0 && alreadydone2 != 1 && alreadydone2 != 2) {
    var alreadydone2 = 3
    var p2 = Math.sqrt(xoneterm)
    var q = Math.sqrt(xtwoterm)
    var r = -g2 / (8*p2*q)
    var s = bq2/(4*aq2)
  }

  if (alreadydone2 == 0 && ipart == 0) {
    if (xthreeterm.toFixed(10) < 0) {
      var alreadydone2 = 4
      var p2 = Math.sqrt(xoneterm)
      var q = Math.sqrt(xtwoterm)
      var r = -g2 / (8*p2*q)
      var s = bq2/(4*aq2)
    } else {
      var alreadydone2 = 5
      var p2 = Math.sqrt(xoneterm.toFixed(10))
      var q = Math.sqrt(xthreeterm.toFixed(10))
      var r = -g2 / (8*p2*q)
      var s = bq2/(4*aq2)
    }
  }

  if (ipart != 0) {
    var p2 = isquareroot(xtwoterm,ipart,0)
    var p2ipart = isquareroot(xtwoterm,ipart,1)
    var q = isquareroot(xthreeterm,-ipart,0)
    var qipart = isquareroot(xthreeterm,-ipart,1)
    var mult = calcmult(p2,p2ipart,q,qipart,0)
    var r = -g2/(8*mult)
    var s = bq2/(4*aq2)
  }

  if (ipart == 0 && xtwoterm.toFixed(10) < 0 && xthreeterm.toFixed(10) < 0) {
    xtwoterm /= -1
    xthreeterm /= -1
    var p2 = 0
    var q = 0
    var p2ipart = Math.sqrt(xtwoterm)
    var qipart = Math.sqrt(xthreeterm)
    var mult = calcmult(p2,p2ipart,q,qipart,0)
    var r = -g2/(8*mult)
    var s = bq2/(4*aq2)
    var ipart = 1
  }

  if (xoneterm.toFixed(10) > 0 && xtwoterm.toFixed(10) < 0 && xthreeterm.toFixed(10) == 0 && ipart == 0) {
    xtwoterm /= -1
    var p2 = Math.sqrt(xoneterm)
    var q = 0
    var p2ipart = 0
    var qipart = Math.sqrt(xtwoterm)
    var mult = calcmult(p2,p2ipart,q,qipart,0)
    var mult2 = calcmult(p2,p2ipart,q,qipart,1)
    var r = -g2/(8*mult)
    if (mult2 != 0) {
      var ripart = g2/(8*mult2)
      var r = 0
    }
    var s = bq2/(4*aq2)
    var ipart = 1
  }

  if (xtwoterm.toFixed(10) == 0 && xthreeterm.toFixed(10) == 0 && ipart == 0) {
    var p2 = Math.sqrt(xoneterm)
    var q = 0
    var r = 0
    var s = bq2/(4*aq2)
  }

  if (ipart == 0) {
    x_1 = eval((p2 + q + r - s).toFixed(10))
    x_2 = eval((p2 - q - r - s).toFixed(10))
    x_3 = eval((-p2 + q - r - s).toFixed(10))
    x_4 = eval((-p2 - q + r - s).toFixed(10))
    return [true,x_1,x_2,x_3,x_4];
  }

  if (perfect == 1) {
    return [true,-bq/4,-bq/4,-bq/4,-bq/4]
    // document.solution.x1.value = ”  ” + -bq/4
    // document.solution.x2.value = ”  ” + -bq/4
    // document.solution.x3.value = ”  ” + -bq/4
    // document.solution.x4.value = ”  ” + -bq/4
    // document.solution.x1i.value = ”  ” +  0
    // document.solution.x2i.value = ”  ” +  0
    // document.solution.x3i.value = ”  ” +  0
    // document.solution.x4i.value = ”  ” +  0
  }

  if (ipart == 0 && xtwoterm.toFixed(10) < 0 && xthreeterm.toFixed(10) < 0) {
    xtwoterm /= -1
    xthreeterm /= -1
    var p2 = 0
    var q = 0
    var p2ipart = Math.sqrt(xtwoterm)
    var qipart = Math.sqrt(xthreeterm)
    var mult = calcmult(p2,p2ipart,q,qipart,0)
    var r = -g2/(8*mult)
    var s = bq2/(4*aq2)
    var ipart = 1
  }

  if (xoneterm.toFixed(10) > 0 && xtwoterm.toFixed(10) < 0 && xthreeterm.toFixed(10) == 0 && ipart == 0) {
    xtwoterm /= -1
    var p2 = Math.sqrt(xoneterm)
    var q = 0
    var p2ipart = 0
    var qipart = Math.sqrt(xtwoterm)
    var mult = calcmult(p2,p2ipart,q,qipart,0)
    var mult2 = calcmult(p2,p2ipart,q,qipart,1)
    var r = -g2/(8*mult)
    if (mult2 != 0) {
      var ripart = g2/(8*mult2)
      var r = 0
    }
    var s = bq2/(4*aq2)
    var ipart = 1
  }

  if (xtwoterm.toFixed(10) == 0 && xthreeterm.toFixed(10) == 0 && ipart == 0) {
    var p2 = Math.sqrt(xoneterm)
    var q = 0
    var r = 0
    var s = bq2/(4*aq2)
  }

  if (ipart != 0) {
    var sols = [];
    var x1  = eval((p2 + q + r - s).toFixed(10))
    var x1i = eval((p2ipart + qipart).toFixed(10))
    if(x1i==0 && x1>=0){sols.push(x1)}
    var x2  = eval((p2 - q - r - s).toFixed(10))
    var x2i = eval((p2ipart - qipart).toFixed(10))
    if(x2i==0 && x2>=0){sols.push(x2)}
    var x3  = eval((-p2 + q - r - s).toFixed(10))
    var x3i = eval((-p2ipart + qipart).toFixed(10))
    if(x3i==0 && x3>=0){sols.push(x3)}
    var x4  = eval((-p2 - q + r - s).toFixed(10))
    var x4i = eval((-p2ipart - qipart).toFixed(10))
    if(x4i==0 && x4>=0){sols.push(x4)}
    return [true,...sols]; // maybe some are good aout of following four
  }

  if (perfectbiquadratic == 1) {
    return [false,0,0,0,0];
    // document.solution.x1i.value = ”  ” + eval(Math.sqrt(cq/2).toFixed(10))
    // document.solution.x2i.value = ”  ” + eval(Math.sqrt(cq/2).toFixed(10))
    // document.solution.x3i.value = ”  -” + eval(Math.sqrt(cq/2).toFixed(10))
    // document.solution.x4i.value = ”  -” + eval(Math.sqrt(cq/2).toFixed(10))
    // document.solution.x1.value = ”  ” + 0
    // document.solution.x2.value = ”  ” + 0
    // document.solution.x3.value = ”  ” + 0
    // document.solution.x4.value = ”  ” + 0
  }
}



function generateExponentialRandom(rate){

    var U = Math.random();
    return -1*Math.log(1-U)/rate;

    // var dummyTime = 0;
    // for(var i = 0; i<numberOfArrivals; i++){
    //     var timeInterval = -1*Math.log(1-Math.random())*meanInterArrivalTime;
    //     dummyTime = dummyTime+timeInterval;
    //     newArrivalTimesArray.push(dummyTime);
    // }
    
}









