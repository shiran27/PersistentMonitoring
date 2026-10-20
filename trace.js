function Trace(traceID,uncertaintyRates,sensingRate,velocity,targetSeq,timeVal,targetStates,costVal) {

    this.traceID = traceID;
    this.uncertaintyRates = [...uncertaintyRates];
    this.sensingRate = sensingRate;
    this.velocity = velocity;

    this.targetSeq = [...targetSeq];    
    this.timeVal = timeVal;  
    this.targetStates = [...targetStates]; 
    this.costVal = costVal;
    

    this.dwellUpdate = function(){
        // dwell till empty the uncertainty at the current target
        // update time, state, cost

        var currentTarget = this.targetSeq[this.targetSeq.length-1];
        var dwellTime = this.targetStates[currentTarget]/(this.sensingRate-this.uncertaintyRates[currentTarget]);

        // targetStates update:
        for(var i = 0; i<this.targetStates.length; i++){
            this.targetStates[i] = this.targetStates[i] + this.uncertaintyRates[i]*dwellTime;
            if(i==currentTarget){
                this.targetStates[i] = 0;
            }
        }

        // cost update 
        var totalArea = this.costVal*this.timeVal; //total area
        for(var i = 0; i<this.targetStates.length; i++){
            if(i==currentTarget){
                totalArea = totalArea + 0.5*this.sensingRate*sq(dwellTime); 
            }else{
                totalArea = totalArea + this.targetStates[i]*dwellTime - 0.5*this.uncertaintyRates[i]*sq(dwellTime);
            }
        }

        // time update
        this.timeVal = this.timeVal + dwellTime;
        this.costVal = totalArea/this.timeVal;
        return this.costVal;

    }


    this.travelUpdate = function(nextTarget,travelTime){
        // this trace continues to add the nextTarget ...
        // update targetsequence, time, state, cost

        // targetSequence update
        this.targetSeq.push(nextTarget);

        // targetStates update:
        for(var i = 0; i<this.targetStates.length; i++){
            this.targetStates[i] = this.targetStates[i] + this.uncertaintyRates[i]*travelTime;
        }

        // cost update 
        var totalArea = this.costVal*this.timeVal; //total area
        for(var i = 0; i<this.targetStates.length; i++){
            totalArea = totalArea + this.targetStates[i]*travelTime - 0.5*this.uncertaintyRates[i]*sq(travelTime);
        }

        // time update
        this.timeVal = this.timeVal + travelTime;
        this.costVal = totalArea/this.timeVal;
    }

    this.expandTrace = function(){
        var expandedTraces = [];
        
        var currentTarget = this.targetSeq[this.targetSeq.length-1];
        var neighbors = targets[currentTarget].neighbors;
        var travelTimes = targets[currentTarget].distancesToNeighbors;

        for(var i = 0; i<neighbors.length; i++){
            if(neighbors[i]!=currentTarget){
                var nextTarget = neighbors[i];
                var travelTime = travelTimes[i]/this.velocity;
                var newTrace = this.copy();
                newTrace.travelUpdate(nextTarget,travelTime);
                expandedTraces.push(newTrace);
            }
        }

        return expandedTraces;
    }


    this.copy = function(){

        var traceID = this.traceID;
        var uncertaintyRates = this.uncertaintyRates;
        var sensingRate = this.sensingRate;
        var velocity = this.velocity;

        var targetSeq = this.targetSeq;
        var timeVal = this.timeVal;  
        var targetStates = this.targetStates; 
        var costVal = this.costVal;
        
        return new Trace(traceID,uncertaintyRates,sensingRate,velocity,targetSeq,timeVal,targetStates,costVal);
    
    }

}


function createATrace(){
    // use the current state of the target network to build a trace (initial condition)
    
    // removeing additional agents.
    if(agents.length>1){
        consolePrint("Removing additional agents.")
        while(agents.length!=1){
            removeAnAgent();
        }    
    }

    var sensingRate = agents[0].sensingRate;

    var targetStates = [];
    var uncertaintyRates = [];
    for(var i = 0; i<targets.length; i++){
        targetStates.push(targets[i].uncertainty);
        uncertaintyRates.push(targets[i].uncertaintyRate);
    }

    var traceID = 0;
    var timeVal = 0;
    var costVal = 0;
    var targetSeq = agents[0].residingTarget;
    var velocity = agents[0].maxLinearVelocity;
    print(targetSeq)
    return new Trace(traceID,uncertaintyRates,sensingRate,velocity,targetSeq,timeVal,targetStates,costVal);

}


function bruteForceExploration(){
    consolePrint("Brute Force Solving Started!")

    var traceBase = [];

    var rootTrace = createATrace();
    rootTrace.dwellUpdate();
    traceBase.push(rootTrace);
    
    var numOfHops = bruteForceDepth;
    var numOfTraces = bruteForceWidth;
    for(var k=0; k<numOfHops; k++){
        print("k="+k);
        var newTraces = [];
        var traceCosts = [];
        for(var l=0; l<traceBase.length; l++){
            newTraceList = traceBase[l].expandTrace();
            for(var m=0; m<newTraceList.length; m++){
                var cost = newTraceList[m].dwellUpdate();
                newTraces.push(newTraceList[m]);
                traceCosts.push(cost);
            }
        }
        
        // newTraces.sort((p,q)=>p.costVal-q.costVal);
        if(newTraces.length>numOfTraces){
            newTraces.sort((p,q)=>p.costVal/Math.pow(p.timeVal,0.4)-q.costVal/Math.pow(q.timeVal,0.4));
            var removedTraces = newTraces.splice(numOfTraces);
            removedTraces.sort((p,q)=>p.costVal-q.costVal);
            removedTraces.splice(numOfTraces)
            newTraces = newTraces.concat(removedTraces);
        }
           
        traceBase = [...newTraces];

        if(newTraces[0].timeVal>periodT && newTraces[numOfTraces].timeVal>periodT){
            print("Period Reached.")
            break;
        }
    }

    traceBase.sort((p,q)=>p.costVal-q.costVal);
    print(traceBase[0])
    print("J: "+traceBase[0].costVal+"; t: "+traceBase[0].timeVal+"; Seq: "+traceBase[0].targetSeq+";")
    
    agents[0].targetSequence = traceBase[0];
    
    consolePrint("Brute Force Solution Found!")
    consolePrint("Cost: "+traceBase[0].costVal.toFixed(4)+"; Time: "+traceBase[0].timeVal.toFixed(4)+";")
    return traceBase;
}


function BruteForceParametersChanged(){
    if(RHCMethod==17){
        bruteForceWidth = Number(document.getElementById("BruteForceParametersWidth").value);
        bruteForceDepth = Number(document.getElementById("BruteForceParametersDepth").value);
    }
    consolePrint("Brute Force Parameters Changed: Depth: "+bruteForceDepth+"; Width: "+bruteForceWidth);
}
