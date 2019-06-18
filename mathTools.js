
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

function showAffinityGraph(method){
    
    var affinityMatrix = [];
    for(var i = 0; i<targets.length; i++){
        var minimumDistancesFound;
        if(method == 0){
            minimumDistancesFound = findMinimumDistancesFrom(i);
        }else{
            minimumDistancesFound = findMinimumMeanCycleUncertaintiesFrom(i);    
        }
        affinityMatrix[i] = minimumDistancesFound[0]; // distances matrix from T_i;
    }
    return affinityMatrix;

}


