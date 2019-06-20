
function sq(val){
    return Math.pow(val,2);
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
        affinityMatrix[i] = minimumDistancesFound[0]; // distances matrix from T_i;
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

    var indexArray = [];
    for(var i = 0; i < N; i++){
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
            }else if(iterationsCount > 100){
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
            if( ((!L1) && L2) || (L1 && (!L2)) ){
                interClusterPaths.push(p);
                paths[p].brokenDueToClustering = true;
            } 
        }
    }



    // creating cycles
    resetCycles(); // to make cycles = [];
    for(var j = 0; j < N; j++){
        cycles.push(new Cycle(j,targetClusters[j]));// deployedAgend,targetSet
        agents[j].assignToTheTarget(targetClusters[j][0]);
    }


    displayClustersMode = true;

}