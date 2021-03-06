
// keep general info regarding the simulation

var agents = [];

var targets = [];

var paths = [];



var problemConfigurationEditMode = false;
var simulationMode = 0; // 0: not started;   1: started;

var agentSelectDropdownUsed = false;
var targetSelectDropdownUsed = false;

var targetDraggingMode = false;
var targetDragging;

var agentDraggingMode = false;
var agentDragging;


var deltaT;
var periodT;
var stepSize;
var simulationTime = 0;
var discreteTimeSteps = 0;

var firstEventTime = 0;
var eventTime = 0; 
var eventCount = 0; 
var simulationFrameRate = 1;

var numberOfUpdateSteps; // gradient update steps
var numberOfUpdateStepsCount = 0; // gradient update steps


var costArrayForPlot = [];
var updateStepCountArray = [];
var costPlotMode = true;

var eventTimeSensitivity = []; // for IPA

var defaultUncertaintyRates = []; // perturbation for boosting purposes
var defaultSensingRates = [];
var targetPrioritizationPolicy; // targetPrioritizationPolicyChanged()


var printMode = false;

// boosting related stuff
var boostingMode = 0;  // starts i normal mode
var boostingMethod;
var bestCostFoundSoFar = Infinity;
var bestThresholdsFoundSoFar = [];

var worstCostFoundSoFar = 100; // for plotting purposes
var boostedUpdateStepCountArray = []; 
var boostedCostArrayToPlot = [];
var boostingCoefficientAlpha;
var thresholdSensitivityMagnitudes = []; // for each agent

var modeSwitchingThresholdAlpha; 
var stepSizeSelectionMethod = 0; // 0- constant, 1 -diminishing, 2- diminishing with margin for error
var numberOfUpdateStepsCountTemp = 0;
// end boosting related stuff


////var cyclicRoutes = [];
var cycles = [];
var RGCComputingMode = 0;
var cycleRefiningParameters = []; // to store i,j,k in 2-opt and 3-opt
var cycleGenerationMethod;
var thresholdGenerationMethod;


var randomNoiseLevelForThresholds;

var similarityMeasureType;
var spectralClusteringMethod;
var neighborhoodWidthForClustering;
var displayClustersMode = false;
var targetClusters = [];
var interClusterPaths = [];
var numberOfKMeansIterations;

var blockingThreshold = 100;

// data plotting needs to be done for period [0,T] 
var dataPlotMode = false;
var agentStateData = []; // NxE matrix residing target label at different times 
var targetStateData = []; // MxE target uncertainty levels at each event time
var eventTimeData = [];

var RHCMethod;
var timeHorizonForRHC = periodT;
var terminalMeanSystemUncertainty; // for plotting purposes with fast-foward

var RHCalpha;
var RHCbeta;
var RHCParameterOverride;
var repeatedRandomTestMode = 0;
var repeatedRandomTestData = "";

var RHCNoiseEnabled = false;
var RHCNoiseA_i;
var RHCNoisevMax;
var RHCNoiseR_iInterval;
var RHCNoiseR_iMagnitude;
var RHCNoiseY_iBoundary;
var RHCNoiseY_iMagnitude;
var RHCNoiseR_j;

// actual agents:
var RHCalpha2;
var RHCvmax; 
var RHCvmaxObserved = 0;
var RHCumaxObserved = 0;
var terminalEnergySpent;
var terminalEnergySpentTemp=0;
var terminalTotalCost;

// RL
var numOfRLIterations = 0;
var RLNormalizationFactor = 8;
var RLDiscountFactor = 0.98;
var RLStepSizeValue = 0.98;
var RLStepSizeType = 0;
var RLRandomPolicyEpsilon = 0;
var RLNumOfEpisodesPerIteration = 1;






function startModifyingProbConfig(){

    problemConfigurationEditMode = true;
    consolePrint("Start modifying the problem configuration !!!");

    document.getElementById("editAgentsWizard").style.display = "inline";
    document.getElementById("editTargetsWizard").style.display = "inline";

    // blank spaces
    document.getElementById("spaceForEditWizard0").style.display = "inline";
    document.getElementById("spaceForEditWizard1").style.display = "inline";
    

    document.getElementById("spaceForRealTimeUncertaintyRateEdit").style.display = "none";
    document.getElementById("spaceForRealTimeSensingRateEdit").style.display = "none";
    document.getElementById("spaceForRealTimeUncertaintyEdit").style.display = "none";
    document.getElementById("spaceForRealTimeThresholdEdit").style.display = "none";
    document.getElementById("spaceForThresholdSensitivities").style.display = "none";
    
    document.getElementById("runSimulationWizard").style.display = "none";
    
}

function finishModifyingProbConfig(){

    problemConfigurationEditMode = false;
    consolePrint("Finished modifying the problem configuration.");

    document.getElementById("editAgentsWizard").style.display = "none";
    document.getElementById("editTargetsWizard").style.display = "none";

    document.getElementById("spaceForEditWizard0").style.display = "none";
    document.getElementById("spaceForEditWizard1").style.display = "none";


    document.getElementById("spaceForRealTimeUncertaintyRateEdit").style.display = "inline";
    document.getElementById("spaceForRealTimeSensingRateEdit").style.display = "inline";
    document.getElementById("spaceForRealTimeUncertaintyEdit").style.display = "inline";
    document.getElementById("spaceForRealTimeThresholdEdit").style.display = "inline";
    document.getElementById("spaceForThresholdSensitivities").style.display = "inline";
    
    document.getElementById("runSimulationWizard").style.display = "inline";



    document.getElementById('customRTUncertaintyRates').innerHTML = "";
    document.getElementById('customRTUncertainty').innerHTML = "";

    for(var i = 0; i<targets.length; i++){

        // updating neighborlist
        targets[i].neighbors = [];
        targets[i].distancesToNeighbors = [];
        for(var j = 0; j<targets.length; j++){
            if(i==j){
                targets[i].neighbors.push(j);
                targets[i].distancesToNeighbors.push(0);
            }else{
                var pid = getPathID(i,j);
                if(paths[pid].isPermenent){
                    targets[i].neighbors.push(j);
                    targets[i].distancesToNeighbors.push(paths[pid].distPath());
                }    
            }
        }

        var indexString = (i+1).toString();
        var HTMLTag = "<h5>Target "+ indexString +":";
        HTMLTag += "<input type='range' min='0' max='15' step='0.2' value='"+targets[i].uncertaintyRate.toFixed(1)+"' class='slider' id='uncertaintyRate"+ indexString +"' onchange='uncertaintyRateChangedRT("+indexString+",this.value)'> <span id='uncertaintyRateDisplay"+ indexString +"'></span></h5>";
        document.getElementById('customRTUncertaintyRates').innerHTML += HTMLTag;         


        var HTMLTag2 = "<td contenteditable='true' id='customRTUncertainty"+indexString+"'>"+targets[i].uncertainty+"</td>";
        document.getElementById('customRTUncertainty').innerHTML += HTMLTag2; 

    }

    // updating second order neighbors of a target
    for(var i = 0; i<targets.length; i++){
        targets[i].neighbors2 = [];
        targets[i].distancesToNeighbors2 = [];
        targets[i].neighborhood = [];
        for(var l = 0; l<targets[i].neighbors.length; l++){
            var j = targets[i].neighbors[l];
            var y_ij = targets[i].distancesToNeighbors[l];
            targets[i].neighbors2[l] = [];
            targets[i].distancesToNeighbors2[l] = [];
            targets[i].neighborhood.push(j); 
            if(i!=j){
                for(var m = 0; m<targets[j].neighbors.length; m++){
                    var k = targets[j].neighbors[m];
                    var y_jk = targets[j].distancesToNeighbors[m];
                    if(j!=k){
                        targets[i].neighbors2[l].push(k);
                        targets[i].distancesToNeighbors2[l].push(y_jk);
                        if(!targets[i].neighbors.includes(k)&&!targets[i].neighborhood.includes(k)){
                            targets[i].neighborhood.push(k);
                        }
                    }
                }
            }
        } 
    }


    // end updating second orderr targets


    document.getElementById('customRTSensingRates').innerHTML  = "";
    document.getElementById('customRTThreshold').innerHTML = "";
    document.getElementById('thresholdSensitivities').innerHTML = "";
    var HTMLTag2 = "<div class='col'></div>";    
    var HTMLTag1 = "";
    for(var a = 0; a<agents.length; a++){
        // print("Error")
        // print(agents[a].sensingRate)
        // print(agents[a].sensingRate.toFixed(1))
        var indexString = (a+1).toString();
        var HTMLTag = "<h5>Agent "+ indexString +":";
        HTMLTag += "<input type='range' min='0' max='20' step='0.1' value='"+agents[a].sensingRate.toFixed(1)+"' class='slider' id='sensingRate"+ indexString +"' onchange='sensingRateChangedRT("+indexString+",this.value)'> <span id='sensingRateDisplay"+ indexString +"'></span></h5>";
        HTMLTag1 += HTMLTag;

        //thresholds per each agent
        HTMLTag2 += "<table class='col table table-bordered matrix' id='customRTThreshold"+(a+1)+"'>";
        agents[a].threshold = [];
        for(var i = 0; i<targets.length; i++){//rows
            agents[a].threshold[i] = [];
            
            HTMLTag2 += "<tr>";
            for(var j = 0; j<targets.length; j++){//columns
                var threshold;
                if( i!=j && !isNeighbors(i,j)){
                    threshold = 10000;    
                }else{
                    threshold = Math.round(100*10*Math.random())/100;
                }
                
                agents[a].threshold[i][j] = threshold;


                HTMLTag2 += "<td contenteditable='true' id='customRTThreshold_"+(i+1)+"_"+(j+1)+"_"+(a+1)+"'>"+threshold+"</td>";
            }
            HTMLTag2 += "</tr>";
        }
        HTMLTag2 += "</table><div class='col'></div>";
    
    }
    document.getElementById('customRTSensingRates').innerHTML += HTMLTag1+"<button class='btn btn-success' type='button' onclick='resetPerturbation();'>Reset Perturbation</button>";
    
    document.getElementById('customRTThreshold').innerHTML += HTMLTag2; 


    // threshold sensitivities table
    var HTMLTag2 = "<div class='col'></div>";    
    for(var a = 0; a<agents.length; a++){

        var indexString = (a+1).toString();
        
        HTMLTag2 += "<table class='col table table-bordered matrix' id='thresholdSensitivities"+(a+1)+"'>";
        agents[a].sensitivityOfThreshold = [];
        for(var i = 0; i<targets.length; i++){//rows
            agents[a].sensitivityOfThreshold[i] = [];
            
            HTMLTag2 += "<tr>";
            for(var j = 0; j<targets.length; j++){//columns
                thresholdSensitivity = 0;
                
                agents[a].sensitivityOfThreshold[i][j] = thresholdSensitivity;

                HTMLTag2 += "<td contenteditable='true' id='thresholdSensitivities_"+(i+1)+"_"+(j+1)+"_"+(a+1)+"'>"+thresholdSensitivity+"</td>";
            }
            HTMLTag2 += "</tr>";
        }
        HTMLTag2 += "</table><div class='col'></div>";
    
    }
    document.getElementById('thresholdSensitivities').innerHTML += HTMLTag2; 

    dataPlotModeChanged();
    RHCParametersChanged();

}


function refreshAll(){
    location.reload(true);
}



function consolePrint(consoleText){
    /*var h4Item = document.getElementById("consoleText");
    h4Item.innerHTML += ">> "+consoleText+"<br>";
    h4Item.scrollTop = h4Item.scrollHeight;*/

    document.getElementById("consoleText").innerHTML += ">> "+consoleText+"<br>";
    document.getElementById("consoleText").scrollTop = document.getElementById("consoleText").scrollHeight;
    //document.getElementById("consoleText").innerHTML = consoleText;
}


function resetPerturbation(){// restore the sening/ uncertainty rates


    for(var i = 0; i<targets.length; i++){
        var val = Number(document.getElementById("uncertaintyRate"+(i+1)).value);
        if(defaultUncertaintyRates[i]!=val){
            uncertaintyRateChangedRT(i+1,defaultUncertaintyRates[i]);
            document.getElementById("uncertaintyRate"+(i+1)).value = defaultUncertaintyRates[i];    
            print("Target "+(i+1)+" Urate set to "+defaultUncertaintyRates[i]);
        } 
        
    }
    for(var a = 0; a<agents.length; a++){
        var val = Number(document.getElementById("sensingRate"+(a+1)).value);
        if(defaultSensingRates[a]!=val){
            sensingRateChangedRT(a+1,defaultSensingRates[a]);
            document.getElementById("sensingRate"+(a+1)).value = defaultSensingRates[a];    
            print("Agent "+(a+1)+" Srate set to "+defaultSensingRates[a]);
        }
        //document.getElementById("sensingRate"+(a+1)).value = defaultSensingRates[a];
        //
    }

}




function mouseDragged() {
    
    if(problemConfigurationEditMode){  
        
        if(agentDraggingMode){
            
            agents[agentDragging].position = new Point2(mouseX,mouseY);
        
        }else if(targetDraggingMode){
            
            targets[targetDragging].position = new Point2(Math.round(mouseX/10)*10,Math.round(mouseY/10)*10);
        
        }
        else{
            
            for (var i = 0; i < agents.length; i++) {

                if(agents[i].clicked()){
                    agentDragging = i;
                    agentDraggingMode = true;
                }
                     
            }
        
            for (var i = 0; i < targets.length; i++) {

                if(targets[i].clicked() && !agentDraggingMode){
                    targetDragging = i;
                    targetDraggingMode = true;
                }
                     
            }
        }   
    }
}



function mouseReleased() {
    
    if(problemConfigurationEditMode){
        if(targetDraggingMode){
            targetDraggingMode = false;
            targets[targetDragging].position = new Point2(Math.round(mouseX/10)*10,Math.round(mouseY/10)*10);
            consolePrint("Dragging Target "+(targetDragging+1)+" Finished");
        }else if(agentDraggingMode){
            agentDraggingMode = false;

            if(targets.length>0){
                var minimumDistanceFound = sqrt(sq(width)+sq(height));
                var closestTargetID = 0;
                for(var i = 0; i<targets.length; i++){
                    var distFound = distP2(new Point2(mouseX,mouseY), targets[i].position);
                    if(distFound<minimumDistanceFound){
                        closestTargetID = i;
                        minimumDistanceFound = distFound;
                    }
                }
                agents[agentDragging].position = targets[closestTargetID].position;

                agents[agentDragging].residingTarget = [closestTargetID];
                agents[agentDragging].initialResidingTarget = closestTargetID;
                
            }else{
                agents[agentDragging].position = new Point2(mouseX,mouseY);
            }
            consolePrint("Dragging Agent "+(agentDragging+1)+" Finished");
        }
    }

}

function mouseClicked(){

    if(problemConfigurationEditMode){

        for (var i = 0; i < paths.length; i++) {
            if(paths[i].clicked()){
                paths[i].isPermenent = !paths[i].isPermenent;
                if(paths[i].isPermenent){
                    consolePrint("Path "+ (i+1) + " between tragets: "+(paths[i].targets[0]+1)+" and "+(paths[i].targets[1]+1)+" is activated.");
                }else{
                    consolePrint("Path "+ (i+1) + " between tragets: "+(paths[i].targets[0]+1)+" and "+(paths[i].targets[1]+1)+" is deactivated.");
                }
            }                
        }

    }

}



function updateInterface(){

    document.getElementById("numberOfTargetsDisplay").innerHTML = targets.length;
    document.getElementById("numberOfAgentsDisplay").innerHTML = agents.length;
    


    if(problemConfigurationEditMode){
        document.getElementById("uncertaintyRateDisplay").innerHTML = document.getElementById("uncertaintyRate").value;
        document.getElementById("sensingRateDisplay").innerHTML = document.getElementById("sensingRate").value;
        document.getElementById("maximumPathLengthDisplay").innerHTML = document.getElementById("maximumPathLength").value;
    }else{

        for(var i = 0; i<targets.length; i++){
            document.getElementById("uncertaintyRateDisplay"+(i+1)).innerHTML = Number(document.getElementById("uncertaintyRate"+(i+1)).value).toFixed(1);
            if(simulationMode==1){
                document.getElementById("customRTUncertainty"+(i+1)).innerHTML = targets[i].uncertainty.toFixed(3);
            }
        }

        for(var i = 0; i<agents.length; i++){
            document.getElementById("sensingRateDisplay"+(i+1)).innerHTML = Number(document.getElementById("sensingRate"+(i+1)).value).toFixed(1);
        }

        document.getElementById("frameRateDisplay").innerHTML = simulationFrameRate.toString();
        document.getElementById("stepSizeDisplay").innerHTML = stepSize.toFixed(3);
        document.getElementById("stepSizeMultiplierDisplay").innerHTML = "10<sup>"+Number(document.getElementById("stepSizeMultiplier").value)+"</sup>";
        document.getElementById("noiseLevelDisplay").innerHTML = randomNoiseLevelForThresholds.toString();
        document.getElementById("neighborhoodWidthForClusteringDisplay").innerHTML = neighborhoodWidthForClustering.toString();
    
        document.getElementById("bestCostFoundSoFar").innerHTML = bestCostFoundSoFar.toFixed(3);
        //document.getElementById("boostingCoefficientAlphaDisplay").innerHTML = boostingCoefficientAlpha.toFixed(3);
        //document.getElementById("modeSwitchingThresholdAlphaDisplay").innerHTML = modeSwitchingThresholdAlpha.toFixed(3);
        
        document.getElementById("blockingThresholdDisplay").innerHTML = blockingThreshold.toFixed(2);
        
        document.getElementById("timeHorizonForRHCDisplay").innerHTML = document.getElementById("timeHorizonForRHC").value;
    
    }

}

function readInitialInterface(){

    deltaT = Number(document.getElementById("deltaT").value);
    periodT = Number(document.getElementById("periodT").value);
    stepSize = Number(document.getElementById("stepSize").value)*Math.pow(10,Number(document.getElementById("stepSizeMultiplier").value));
    numberOfUpdateSteps = Number(document.getElementById("numberOfUpdateSteps").value);
    randomNoiseLevelForThresholds = Number(document.getElementById("noiseLevel").value);
    
    cycleGenerationMethod = document.getElementById("cycleGenerationMethod").checked;
    thresholdGenerationMethod = document.getElementById("thresholdGenerationMethod").checked;
    
    similarityMeasureType = Number(document.getElementById("similarityMeasureTypeDropdown").value);
    spectralClusteringMethod = Number(document.getElementById("spectralClusteringMethodDropdown").value);
    neighborhoodWidthForClustering = Number(document.getElementById("neighborhoodWidthForClustering").value);
    numberOfKMeansIterations = Number(document.getElementById("numOfKMeansIterations").value);
    

    boostingMethodChanged();
    boostingCoefficientAlpha = Number(document.getElementById("boostingCoefficientAlpha").value);
    modeSwitchingThresholdAlpha = Number(document.getElementById("modeSwitchingThresholdAlpha").value);
    boostingMethod = Number(document.getElementById("boostingMethodDropdown").value);
    stepSizeSelectionMethod = Number(document.getElementById("stepSizeSelectionMethodDropdown").value);
    
    blockingThreshold = Number(document.getElementById("blockingThreshold").value);

    adjustNeighborhoodWidthForClusteringRange();
    targetPrioritizationPolicyChanged();

    dataPlotMode = document.getElementById("dataPlotModeCheckBox").checked;
    dataPlotModeChanged();

    RHCMethod = Number(document.getElementById("RHCMethodDropdown").value);
    timeHorizonForRHC = Number(document.getElementById("timeHorizonForRHC").value);

    RHCvmax = Number(document.getElementById("RHCvmax").value);
    var normalizedAlpha = Number(document.getElementById("RHCalpha2").value);
    // RHCalpha2 = (normalizedAlpha/(27*(1-normalizedAlpha)))*Math.pow(300/Math.sqrt(targets.length),3)/Math.pow(RHCvmax,5);
    RHCalpha2 = RHCalpha2 = (normalizedAlpha/(27*(1-normalizedAlpha)))*Math.pow(600/Math.sqrt(targets.length),2)/Math.pow(RHCvmax,4);
    RHCMethodChanged();
    RHCNoiseEnable();
    RHCNoiseR_iIntervalChanged();
}

function displayThresholdSensitivities(){ 

    for(var a = 0; a<agents.length; a++){
        for(var i = 0; i<targets.length; i++){//rows
            for(var j = 0; j<targets.length; j++){//columns
                var thresholdSensitivity = agents[a].sensitivityOfThreshold[i][j];
                document.getElementById("thresholdSensitivities_"+(i+1)+"_"+(j+1)+"_"+(a+1)).innerHTML = thresholdSensitivity.toFixed(3);
            }
        }    
    }

    
    ////consolePrint("Obtained threshold sensitivities were loaded to the display.")
}



function updateRTThresholdValuesRandom(){ 

    for(var a = 0; a<agents.length; a++){

        for(var i = 0; i<targets.length; i++){//rows
            
            for(var j = 0; j<targets.length; j++){//columns 

                var threshold;
                if( i!=j && !isNeighbors(i,j)){
                    threshold = 10000;    
                }else{
                    threshold = Math.round(100*10*Math.random())/100;
                }
                agents[a].threshold[i][j] = threshold;
                document.getElementById("customRTThreshold_"+(i+1)+"_"+(j+1)+"_"+(a+1)).innerHTML = threshold.toString();
   
            }
            
        }

    }

    consolePrint("All thresholds were randomly updated.");

}


function updateRTThresholdValuesUsingRHC(){
    resetSimulation();
    simulateHybridSystemFast();
    var sumError = 0
    if(agents.length==1){
        for(var a = 0; a<agents.length; a++){
            var sumVal = agents[a].learnThresholdControlPolicy();
            sumError = sumError + sumVal;
        }
        displayThresholds();
    }else{
        // agents learn cooperatively to prevent merging
        for(var a = 0; a<agents.length; a++){
            var sumVal = agents[a].learnThresholdControlPolicyJointly();
            sumError = sumError + sumVal;
        }
        displayThresholds();
        // displayThresholds();
    }
    resetSimulation();
    sumError = sumError/agents.length;
    consolePrint("Learning TCPs Finished with Mean Error: "+sumError.toFixed(6));
    
}


function learnClassifiersUsingRHC(){

    resetSimulation();
    simulateHybridSystemFast();
    var sumError = 0;
    if(agents.length>0){

        for(var a = 0; a<agents.length; a++){
            var sumVal = agents[a].learnClassifierCoefficients();
            sumError = sumError + sumVal;
        }
    }else{
        // agents learn cooperatively to prevent merging
        // for(var a = 0; a<agents.length; a++){
        //     agents[a].learnThresholdControlPolicyJointly();
        // }
        // displayThresholds();
        // displayThresholds();
    }
    resetSimulation();
    sumError = sumError/agents.length;
    consolePrint("Learning Classifiers Finished with Mean Error: "+sumError.toFixed(6))

}



function learnClassifiersUsingSyntheticData(){

    var numOfSamples = Number(document.getElementById("numOfSyntheticDatePoints").value);
    if(RHCMethod==15||RHCMethod==14){
        numOfRLIterations=0;
        tuneRLBasedEDRHC(numOfSamples);
        return;
    }

    
    // remove additional agents keeping only one
    var agentsBackup = agents.map(a => Object.assign({}, a));
    //var agentsBackup = [...agents]; // this does not copy the array of objects
    var targetUncertaintyBackup = [];
    for(var i=0; i<targets.length; i++){
        targetUncertaintyBackup.push(targets[i].uncertainty);
    }
    agents = [agents[0]];

    // for all the targets,
        // move the agent to a certain target
        // adjust the neighbor target uncertainties
            // record the decition made the RHC
            // add the data point (as trajectory data?)
    //learn from data points
    // var numOfSamples = 100;
    // var upperbound = 20; // 80 in 1A 9T case
    var upperbound = 20;
    for(var i = 0; i<targets.length; i++){
        agents[0].assignToTheTarget(i);
        targets[i].uncertainty = 0;
        
        for(var k = 0; k<numOfSamples; k++){
            // set neighbor target uncertainties randomly
            var dataPacket = [i]; // replace the first element with j^* found using RHC
            for(var jInd = 0; jInd<targets[i].neighbors.length; jInd++){
                var j = targets[i].neighbors[jInd];
                if(j==i){
                    dataPacket.push(0); 
                }else{
                    var u = Math.random();  
                    var R_j = upperbound*(1-Math.sqrt(1-u)); // pdf of R_j decreases linearly
                    // var R_j = upperbound*Math.random();  
                    targets[j].uncertainty = R_j;
                    dataPacket.push(R_j);
                }             
            }

            // solve the RHCP
            var ans1 = agents[0].solveOP3(i); 
            var jStar = ans1[0];

            dataPacket[0] = jStar;
            agents[0].trajectoryData[i].push(dataPacket);
        }
    }

    // learn
    agents[0].learnClassifierCoefficients();

    // exchange the coefficients to all the agents
    var classifierCoefficients = [...agents[0].classifierCoefficients];
    for(var a = 0; a<agentsBackup.length; a++){
        agentsBackup[a].classifierCoefficients = [...classifierCoefficients];
    }
    
    // reset
    agents = agentsBackup.map(a => Object.assign({}, a));
    for(var i = 0; i<targets.length; i++){
        targets[i].uncertainty = targetUncertaintyBackup[i];
    }

}



function rollbackToBestPerformingThresholdsFoundSoFar(){
    print("Best thresholds: ");
    print(bestThresholdsFoundSoFar);
    print("Current thresholds: ");
    // cloning multi-dimentional arrays is a tricky thing! - better to do element wise
    for(var a = 0; a<agents.length; a++){
        print(agents[a].threshold);
        for(var i = 0; i<targets.length; i++){//rows
            for(var j = 0; j<targets.length; j++){//columns
                agents[a].threshold[i][j] = bestThresholdsFoundSoFar[a][i][j];
            }
        }
    }

    displayThresholds();
    
}


function updateRTThresholdValues(){

    for(var a = 0; a<agents.length; a++){

        for(var i = 0; i<targets.length; i++){//rows
            
            for(var j = 0; j<targets.length; j++){//columns
                
                var threshold = Number(document.getElementById("customRTThreshold_"+(i+1)+"_"+(j+1)+"_"+(a+1)).innerHTML);
                agents[a].threshold[i][j] = threshold;

            }
            
        }
        
    }

    consolePrint("Thresholds were updated.");

}


function updateRTUncertaintyValuesRandom(){
    for(var i = 0; i<targets.length; i++){
        var uncertainty = Math.round(10*25*Math.random())/10;
        document.getElementById("customRTUncertainty"+(i+1)).innerHTML = uncertainty.toString();
        targets[i].uncertainty = uncertainty;
    }

    consolePrint("Target uncertainty values were randomly updated.");
}

function updateRTUncertaintyValues(){// read the matrix and update
    for(var i = 0; i<targets.length; i++){
        targets[i].uncertainty = Number(document.getElementById("customRTUncertainty"+(i+1)).innerHTML);
        if(simulationMode==0){
            targets[i].initialUncertainty = targets[i].uncertainty;
        }
    }
    consolePrint("Target uncertainty values were updated.");
}

function setToZeroRTUncertaintyValues(){
    for(var i = 0; i<targets.length; i++){
        targets[i].uncertainty = 0;
        document.getElementById("customRTUncertainty"+(i+1)).innerHTML = targets[i].uncertainty.toString();
        if(simulationMode==0){
            targets[i].initialUncertainty = targets[i].uncertainty;
        }
    }
}



function sensingRateChangedRT(index,val){
    consolePrint("Agent "+index+"'s sensing rate changed to "+val+".")
    agents[index-1].sensingRate = val;
}


function uncertaintyRateChangedRT(index,val){
    consolePrint("Target "+index+"'s uncertainty rate changed to "+val+".")
    targets[index-1].uncertaintyRate = val;
}

function randomNoiseLevelChanged(val){
    consolePrint("Random noise level, which is to be added once perturbed, changed to "+val+".");
    randomNoiseLevelForThresholds = Number(document.getElementById("noiseLevel").value);

}

function boostingCoefficientAlphaChanged(val){
    consolePrint("Boosting coefficient alpha changed to "+val+".");
    boostingCoefficientAlpha = val;
}

function modeSwitchingThresholdAlphaChanged(val){
    consolePrint("Mode switching (i.e. between: Normal and Boosting) threshold changed to "+val+".");
    modeSwitchingThresholdAlpha = val;
}

function blockingThresholdChanged(val){
    consolePrint("Blocking threshold changed to "+val+".");

    blockingThreshold = val;
    generateThresholdsFromRoutes();
}


function timeHorizonForRHCChanged(val){
    consolePrint("Time horizon changed to: "+val+".")
    timeHorizonForRHC = val;
}

function RHCParametersChanged(){
    
    RHCParameterOverride = document.getElementById("RHCParamterOverrideCB").checked;
    if(RHCParameterOverride){
        document.getElementById("RHCalpha").disabled = false;
        document.getElementById("RHCbeta").disabled = false;                
    }else{
        document.getElementById("RHCalpha").disabled = true;        
        document.getElementById("RHCbeta").disabled = true;        
    }    

    
    RHCFixalpha = document.getElementById("RHCParamterFixAlphaCB").checked;
    if(RHCFixalpha){
        document.getElementById("RHCalpha2").disabled = true;
    }else{
        document.getElementById("RHCalpha2").disabled = false;
    }



    RHCalpha = Number(document.getElementById("RHCalpha").value);
    RHCbeta = Number(document.getElementById("RHCbeta").value);
    

    if(RHCMethod==4){
        document.getElementById("RHCalphaDisplay").innerHTML = RHCalpha;
        if(!RHCParameterOverride){
            document.getElementById("RHCalphaDisplay").innerHTML = "N/A";
        }
        consolePrint("RHC parameters: Alpha = "+RHCalpha+".");
    }else if(RHCMethod==7){
        document.getElementById("RHCalphaDisplay").innerHTML = RHCalpha;
        document.getElementById("RHCbetaDisplay").innerHTML = RHCbeta;
        if(!RHCParameterOverride){
            document.getElementById("RHCalphaDisplay").innerHTML = "N/A";
            document.getElementById("RHCbetaDisplay").innerHTML = "N/A";
        }
        consolePrint("RHC parameters: Alpha = "+RHCalpha+", Beta = "+RHCbeta+".");
    }else if(RHCMethod==8||RHCMethod==9||RHCMethod==10||RHCMethod==11){
        
        if(RHCFixalpha){
            RHCvmax = Number(document.getElementById("RHCvmax").value);    
        }else{
            var normalizedAlpha = Number(document.getElementById("RHCalpha2").value);
            RHCvmax = Number(document.getElementById("RHCvmax").value);
            //RHCalpha2 = (normalizedAlpha/(27*(1-normalizedAlpha)))*Math.pow(300/Math.sqrt(targets.length),3)/Math.pow(RHCvmax,5);
            RHCalpha2 = (normalizedAlpha/(27*(1-normalizedAlpha)))*Math.pow(600/Math.sqrt(targets.length),2)/Math.pow(RHCvmax,4);
        }
        
        document.getElementById("RHCalpha2Display").innerHTML = RHCalpha2.toExponential(2);
        document.getElementById("RHCvmaxDisplay").innerHTML = RHCvmax;
        consolePrint("RHC parameters: Scaling Factor: Alpha = "+RHCalpha2.toExponential(2)+", Agent Velocity: v_max = "+RHCvmax+".");
    }else{
        consolePrint("RHC parameters: Alpha = "+RHCalpha+", Beta = "+RHCbeta+".");
    }


    RLParametersChanged()
    
}



function RLParametersChanged(){
    if(RHCMethod==15){
        numOfRLIterations = 0;
        document.getElementById("RLParametersNorm").disabled = false;
        RLNormalizationFactor = Number(document.getElementById("RLParametersNorm").value);
        RLDiscountFactor = Number(document.getElementById("RLParametersDisc").value);
        RLStepSizeValue = Number(document.getElementById("RLParametersStepVal").value);
        RLStepSizeType = Number(document.getElementById("RLParametersStepType").value);
        RLRandomPolicyEpsilon = Number(document.getElementById("RLParametersRand").value);
        RLNumOfEpisodesPerIteration = Number(document.getElementById("RLParametersReal").value);
    }else if(RHCMethod==14){ // same as before
        numOfRLIterations = 0;
        document.getElementById("RLParametersNorm").disabled = true;
        RLDiscountFactor = Number(document.getElementById("RLParametersDisc").value);
        RLStepSizeValue = Number(document.getElementById("RLParametersStepVal").value);
        RLStepSizeType = Number(document.getElementById("RLParametersStepType").value);
        RLRandomPolicyEpsilon = Number(document.getElementById("RLParametersRand").value);
        RLNumOfEpisodesPerIteration = Number(document.getElementById("RLParametersReal").value);
    }
}



function stepSizeSelectionMethodChanged(){
    stepSizeSelectionMethod = Number(document.getElementById("stepSizeSelectionMethodDropdown").value);

    if(stepSizeSelectionMethod==0){
        consolePrint("Step size selection method: Constant with value: "+stepSize+".");
    }else if(stepSizeSelectionMethod==1){
        consolePrint("Step size selection method: Diminishing according to: "+stepSize+"/SQRT(k).");
    }else if(stepSizeSelectionMethod==2){
        consolePrint("Step size selection method: Square summable: "+stepSize+"/("+stepSize+"+k).");
    }else{
        consolePrint("Step size selection method: Constant with value "+stepSize+".");
    }
}


function sensingRateChanged(val){
    if(agentSelectDropdownUsed){
        agentSelectDropdownUsed = false;
    }else{
        var val1 = Number(document.getElementById("agentSelectDropdown").value); 
        if(isNaN(val1)){
            for(var i = 0; i<agents.length; i++){// all agents reset to default
                agents[i].sensingRate = Number(val);
            }
        }else{
            agents[val1-1].sensingRate = Number(val);
        }
    }
}

function uncertaintyRateChanged(val){
    if(targetSelectDropdownUsed){
        targetSelectDropdownUsed = false;
    }else{
        var val1 = Number(document.getElementById("targetSelectDropdown").value); 
        if(isNaN(val1)){
            for(var i = 0; i<targets.length; i++){// all agents reset to default
                targets[i].uncertaintyRate = Number(val);
            }
        }else{
            targets[val1-1].uncertaintyRate = Number(val);
        }
    }
}


function maximumPathLengthChanged(){
    val = Number(document.getElementById("maximumPathLength").value);
    print(val);
    for(var i = 0; i<paths.length; i++){
        var T_i = paths[i].targets[0];
        var T_j = paths[i].targets[1];
        if(distP2(targets[T_i].position,targets[T_j].position)>val){
            paths[i].isPermenent = false;
        }else if(distP2(targets[T_i].position,targets[T_j].position)<val){
            paths[i].isPermenent = true;
        }
    }
}



function targetPrioritizationPolicyChanged(){
    var cb1 = document.getElementById("minimumDistancePolicyCheckBox").checked;
    var cb2 = document.getElementById("maximumUncertaintyPolicyCheckBox").checked;
    if(cb1 && cb2){
        targetPrioritizationPolicy = 3;
    }else if(cb2){
        targetPrioritizationPolicy = 2;
    }else{
        targetPrioritizationPolicy = 1;
    }
    print("targetPrioritizationPolicy: "+targetPrioritizationPolicy);
    
}

function  cycleGenerationMethodChanged(){

    cycleGenerationMethod = document.getElementById("cycleGenerationMethod").checked;
    
    if(cycleGenerationMethod){
        consolePrint("Multiple visits to an any target during the cycle is allowed!");
    }else{
        consolePrint("Multiple visits to an any target during the cycle is not allowed!");
    }
}


function  thresholdGenerationMethodChanged(){
    
    thresholdGenerationMethod = document.getElementById("thresholdGenerationMethod").checked;

    if(thresholdGenerationMethod){
        consolePrint("Thresholds (for the cycles found) will be selected according to the steady state theoretical results!");
    }else{
        consolePrint("Thresholds will be selected from the set {0, Blocking Threshold, 10000}.");
    }
}

function  dataPlotModeChanged(){
    dataPlotMode = document.getElementById("dataPlotModeCheckBox").checked;
    if(dataPlotMode){

        targetStateData = [];
        for(var i = 0; i<targets.length; i++){
            targetStateData.push([]);
        }
        
        agentStateData = [];
        for(var a = 0; a<agents.length; a++){
            agentStateData.push([]);
        }

        eventTimeData = [];
        //consolePrint("Enabled generation of additional data plots.");

        // clearing individual agent data : for TCP learning and classifier based learning
        for(var a = 0; a<agents.length; a++){
            agents[a].trajectoryData = [];
            //agents[a].classifierCoefficients = [];
            agents[a].rewardSequence = [];
            agents[a].rewardDataTemp = [];
            if(numOfRLIterations==0){
                agents[a].actionValueData = [];
            }
            
            for(var i = 0; i<targets.length; i++){
                agents[a].trajectoryData.push([]);
                //agents[a].classifierCoefficients.push([]);
                
                if(numOfRLIterations==0){
                    agents[a].actionValueData.push([]);
                    for(var j = 0; j<targets.length; j++){
                        // [overall mean, episode mean, requence of action values obseved]
                        agents[a].actionValueData[i].push([0,0]); 
                    }
                }
            }
        }
        // end clearing individual agent date : for TCP learning        


    }else{
        //consolePrint("Disabled generation of additional data plots.");        
    }
}

function agentSelectDropdownEvent(){
    
    var val = Number(document.getElementById("agentSelectDropdown").value); 
    if(isNaN(val)){
        print("Resetting all agents");
        for(var i = 0; i<agents.length; i++){// all agents reset to default
            agents[i].sensingRate = Number(document.getElementById("sensingRate").value);
        }
        document.getElementById("sensingRate").value = Number(document.getElementById("sensingRate").value);
        
    }else{// just display
        print("Showing Agent "+val+" config");
        document.getElementById("sensingRate").value = agents[val-1].sensingRate;
    }

    consolePrint("Selected agent changed.");
    agentSelectDropdownUsed = true;

}

function targetSelectDropdownEvent(){
    
    var val = Number(document.getElementById("targetSelectDropdown").value); 
    if(isNaN(val)){
        print("Resetting all targets");
        for(var i = 0; i<targets.length; i++){// all agents reset to default
            targets[i].uncertaintyRate = Number(document.getElementById("uncertaintyRate").value);
        }
        document.getElementById("uncertaintyRate").value = Number(document.getElementById("uncertaintyRate").value);
        
    }else{// just display
        print("Showing Target "+val+" config");
        document.getElementById("uncertaintyRate").value = targets[val-1].uncertaintyRate;
        
    }

    consolePrint("Selected target changed.");
    targetSelectDropdownUsed = true;
    
}




function deltaTChanged(val){
    if(!isNaN(val) && val > 0){
        deltaT = val;
        consolePrint("Continuous time step size changed to: "+val+".");
    }
}

function periodTChanged(val){
    if(!isNaN(val) && val > 0){
        periodT = val;
        consolePrint("Time period T (of the objective function) changed to: "+val+".");
    }
}

function stepSizeChanged(){
    
    var stepSizeRaw = Number(document.getElementById("stepSize").value);
    var stepSizeMultiplier = Number(document.getElementById("stepSizeMultiplier").value);

    stepSize = stepSizeRaw * Math.pow(10,stepSizeMultiplier);

    stepSizeSelectionMethodChanged();

    ////consolePrint("Step size of the gradient descent scheme changed to: "+stepSize+".");
    
    document.getElementById("stepSizeDisplay").innerHTML = stepSize.toString();
    document.getElementById("stepSizeMultiplierDisplay").innerHTML = "10<sup>"+stepSizeMultiplier+"</sup>";
}




function boostingMethodChanged(){

    boostingMethod = Number(document.getElementById('boostingMethodDropdown').value);
    
    if(boostingMode==1){// if we were inside a boosting mode, go back to normal mode
        initiateForcedModeSwitch();
    }

    if(boostingMethod==0){
        consolePrint("No boosting method will be used.");
    }else if(boostingMethod==1){
        document.getElementById('boostingCoefficientAlpha').value = '1';
        consolePrint("Neighbor boosting method will be used.");
    }else if(boostingMethod==2){
        document.getElementById('boostingCoefficientAlpha').value = '0.5';
        consolePrint("Arc-boosting method will be used.");
    }else if(boostingMethod==3){
        consolePrint("Random Perturbation method will be used.");
    }else if(boostingMethod==4){
        document.getElementById('boostingCoefficientAlpha').value = '0.3';
        consolePrint("Split Boosting method will be used.");
    }else if(boostingMethod==5){
        document.getElementById('boostingCoefficientAlpha').value = '0.8';
        consolePrint("Exploration Boosting method will be used.");
    }

}

function RHCMethodChanged(){
    RHCMethod = Number(document.getElementById("RHCMethodDropdown").value);
    if(RHCMethod==0){
        consolePrint("Receding horizon control: Disabled. Agent Control via Thresholds.")
    }else if(RHCMethod==1){
        consolePrint("Receding horizon control: One Steap Ahead Method Enabled.");
    }else if(RHCMethod==2){
        consolePrint("Receding horizon control: Two Steps Ahead Method Enabled.");
    }else if(RHCMethod==3){
        consolePrint("Event Driven Receding Horizon Control.");
    }else if(RHCMethod==4){
        consolePrint("Event Driven Receding Horizon Control-Alpha.");
    }else if(RHCMethod==5){
        consolePrint("Event Driven Receding Horizon Control with a Fixed Horizon.");      
    }else if(RHCMethod==6){
        consolePrint("Event Driven Receding Horizon Control with Two Steps Ahead.");
    }else if(RHCMethod==7){
        consolePrint("Event Driven Receding Horizon Control with Two Steps Ahead-Alpha,Beta.");
    }else if(RHCMethod==8){
        consolePrint("Event Driven Optimal Receding Horizon Control with First Order Agents (Old Method).");
    }else if(RHCMethod==9){
        consolePrint("Event Driven Optimal Receding Horizon Control with Second Order Agents.");
    }else if(RHCMethod==10){
        consolePrint("Event Driven Optimal Receding Horizon Control with First Order Agents of Type 2.");
    }else if(RHCMethod==11){
        consolePrint("Event Driven Optimal Receding Horizon Control with First Order Agents of Type 3.");
    }else if(RHCMethod==12){
        consolePrint("Agent Control Through Classifiers.");
    }else if(RHCMethod==13){
        consolePrint("Random Agent Control.");
    }else if(RHCMethod==14){
        consolePrint("Random Agent Control with Reinforcement Learning.");
    }else if(RHCMethod==15){
        consolePrint("Event Driven Receding Horizon Control with Reinforcement Learning.");
    }


    if(RHCMethod==4){
        var x = document.getElementById("RHCalphaDiv");
        x.style.display = "block";
        
        var x = document.getElementById("RHCbetaDiv");
        x.style.display = "none";
        RHCParametersChanged();

        var x = document.getElementById("plotCostVsParameterDropDown");
        if(Number(x.value)==3){
            x.value = 0;
        }
        x.options[1].disabled = false;  
        x.options[2].disabled = false;
        x.options[3].disabled = true;
        x.options[5].disabled = true;
        x.options[6].disabled = true;
        

    }else if(RHCMethod==7){
        var x = document.getElementById("RHCalphaDiv");
        x.style.display = "block";
        
        var x = document.getElementById("RHCbetaDiv");
        x.style.display = "block";
        RHCParametersChanged();
        
        var x = document.getElementById("plotCostVsParameterDropDown");
        x.options[1].disabled = false;  
        x.options[2].disabled = false;
        x.options[3].disabled = false;
        x.options[5].disabled = true;
        x.options[6].disabled = true;
    }else{
        var x = document.getElementById("RHCalphaDiv");
        x.style.display = "none";
        
        var x = document.getElementById("RHCbetaDiv");
        x.style.display = "none";

        var x = document.getElementById("plotCostVsParameterDropDown");
        if(Number(x.value)>=2){
            x.value = 0;
        }

        x.options[2].disabled = true;
        x.options[3].disabled = true;
        x.options[5].disabled = true;
        x.options[6].disabled = true;

        if(RHCMethod==0){
            x.options[1].disabled = true;
            x.value = 0;            
        }else{
            x.options[1].disabled = false;            
        }

    }

    if(RHCMethod==8 || RHCMethod==9 || RHCMethod==10 || RHCMethod==11){
        var x = document.getElementById("RHCalpha2Div");
        x.style.display = "block";
        
        var x = document.getElementById("RHCbeta2Div");
        x.style.display = "block";

        var x = document.getElementById("costDisplayMenu1");
        x.style.display = "none";

        var x = document.getElementById("costDisplayMenu2");
        x.style.display = "block";

        var x = document.getElementById("plotCostVsParameterDropDown");
        x.options[5].disabled = RHCMethod==9;
        x.options[6].disabled = false;

        RHCParametersChanged();  
        plotCostVsParameterDropDownChanged();    
    }else{
        var x = document.getElementById("RHCalpha2Div");
        x.style.display = "none";
        
        var x = document.getElementById("RHCbeta2Div");
        x.style.display = "none";

        var x = document.getElementById("costDisplayMenu1");
        x.style.display = "block";

        var x = document.getElementById("costDisplayMenu2");
        x.style.display = "none";

        plotCostVsParameterDropDownChanged();    
    }


    if(RHCMethod==15||RHCMethod==14){
        var x = document.getElementById("RLParametersMenu");
        x.style.display = "block";
        RLParametersChanged();
    }else{
        var x = document.getElementById("RLParametersMenu");
        x.style.display = "none";
    }
    
}


function plotCostVsParameterDropDownChanged(){
    var x = Number(document.getElementById("plotCostVsParameterDropDown").value);
    if(x==0){
        document.getElementById("plotCostVsParameterStart").value = 100;
        document.getElementById("plotCostVsParameterRes").value = 100;
        document.getElementById("plotCostVsParameterEnd").value = 5000;
    }else if(x==1){
        document.getElementById("plotCostVsParameterStart").value = 5;
        document.getElementById("plotCostVsParameterRes").value = 5;
        document.getElementById("plotCostVsParameterEnd").value = 250;
    }else if(x==2 || x==3){
        document.getElementById("plotCostVsParameterStart").value = 0;
        document.getElementById("plotCostVsParameterRes").value = 0.05;
        document.getElementById("plotCostVsParameterEnd").value = 1;
    }else if(x==5){
        document.getElementById("plotCostVsParameterStart").value = 10;
        document.getElementById("plotCostVsParameterRes").value = 10;
        document.getElementById("plotCostVsParameterEnd").value = 100;
    }else if(x==6){
        document.getElementById("plotCostVsParameterStart").value = 0.05;
        document.getElementById("plotCostVsParameterRes").value = 0.05;
        document.getElementById("plotCostVsParameterEnd").value = 0.95;
    }

    if(x==4){
        document.getElementById("plotCostVsParameterStart").value = 1;
        document.getElementById("plotCostVsParameterStart").disabled = true;
        document.getElementById("plotCostVsParameterRes").value = 1;
        document.getElementById("plotCostVsParameterRes").disabled = true;
        document.getElementById("plotCostVsParameterEnd").value = 50;
    }else{
        document.getElementById("plotCostVsParameterStart").disabled = false;
        document.getElementById("plotCostVsParameterRes").disabled = false;   
    }



}


function RHCNoiseEnable(){

    RHCNoiseEnabled = document.getElementById("RHCNoiseEnableCB").checked;
    if(RHCNoiseEnabled){
        document.getElementById("RHCNoiseA_iMenu").style.display = "block";
        RHCNoiseA_i = Number(document.getElementById("RHCNoiseA_i").value);
        document.getElementById("RHCNoiseA_iDisplay").innerHTML = RHCNoiseA_i;

        document.getElementById("RHCNoisev_MaxMenu").style.display = "block";
        RHCNoisev_Max = Number(document.getElementById("RHCNoisev_Max").value);
        document.getElementById("RHCNoisev_MaxDisplay").innerHTML = RHCNoisev_Max;
        
        document.getElementById("RHCNoiseR_iMenu").style.display = "block";
        RHCNoiseR_iInterval = Number(document.getElementById("RHCNoiseR_iInterval").value);
        document.getElementById("RHCNoiseR_iIntervalDisplay").innerHTML = RHCNoiseR_iInterval;
        RHCNoiseR_iMagnitude = Number(document.getElementById("RHCNoiseR_iMagnitude").value);
        document.getElementById("RHCNoiseR_iMagnitudeDisplay").innerHTML = RHCNoiseR_iMagnitude;
    
        document.getElementById("RHCNoiseY_iMenu").style.display = "block";
        RHCNoiseY_iBoundary = Number(document.getElementById("RHCNoiseY_iBoundary").value);
        document.getElementById("RHCNoiseY_iBoundaryDisplay").innerHTML = RHCNoiseY_iBoundary;
        RHCNoiseY_iMagnitude = Number(document.getElementById("RHCNoiseY_iMagnitude").value);
        document.getElementById("RHCNoiseY_iMagnitudeDisplay").innerHTML = RHCNoiseY_iMagnitude;

        document.getElementById("RHCNoiseR_jMenu").style.display = "block";
        RHCNoiseR_j = Number(document.getElementById("RHCNoiseR_j").value);
        document.getElementById("RHCNoiseR_jDisplay").innerHTML = RHCNoiseR_j;

    }else{
        document.getElementById("RHCNoiseA_iMenu").style.display = "none";
        document.getElementById("RHCNoisev_MaxMenu").style.display = "none";
        document.getElementById("RHCNoiseR_iMenu").style.display = "none";
        document.getElementById("RHCNoiseY_iMenu").style.display = "none";
        document.getElementById("RHCNoiseR_jMenu").style.display = "none";
    } 
}



function RHCNoiseR_iIntervalChanged(){
    RHCNoiseEnable();
    //need to update the event time arrays

    for(var i=0;i<targets.length;i++){
        targets[i].RHCNoiseR_iEventTimes = [];
        var dummyTime = simulationTime;
        while(dummyTime<periodT){
            var timeInterval = -1*Math.log(1-Math.random())*RHCNoiseR_iInterval;
            dummyTime = dummyTime+timeInterval;
            if(dummyTime<periodT){
                dummyTime = Math.round(100*dummyTime)/100;
                targets[i].RHCNoiseR_iEventTimes.push(dummyTime);
            }
        }
        targets[i].RHCNoiseR_iEventIndex = 0;
        ////print(targets[i].RHCNoiseR_iEventTimes)
    }

}









function initiateForcedModeSwitch(){
    if(boostingMode == 0){
        consolePrint("Boosting Activated!");
        boostingMode = 1;
        document.getElementById("forcedModeSwitchButton").innerHTML = "Norm."; 
        document.getElementById("forcedModeSwitchButton").classList.add('btn-success');
        document.getElementById("forcedModeSwitchButton").classList.remove('btn-danger'); 
        numberOfUpdateStepsCountTemp = 0;
    }else if(boostingMode == 1){
        consolePrint("Boosting Deactivated");
        boostingMode = 0;
        document.getElementById("forcedModeSwitchButton").innerHTML = "Boost";
        document.getElementById("forcedModeSwitchButton").classList.add('btn-danger');
        document.getElementById("forcedModeSwitchButton").classList.remove('btn-success');
        numberOfUpdateStepsCountTemp = 0
    }
}




function simulateHybridSystem(){ // run the hybrid system indefinitely in real-time while displaying

    if(simulationMode>0){
        simulationMode = 0;
        document.getElementById("simulateHybridSystemButton").innerHTML = "<i class='fa fa-play' aria-hidden='true'></i>"; 
        consolePrint("Hybrid system simulation paused.");
    }else{
        if(RHCMethod==0){//disabled
            consolePrint("Initiated simulating the system using the threshold based controller.");
            simulationMode = 1;
            ////simulationTime = 0;
            ////discreteTimeSteps = 0;
        }
        else if(RHCMethod<3){
            if(RHCMethod==1){
                consolePrint("Initiated simulating the system using 'one-step-ahead' receding horizon controller.");
            }else if(RHCMethod==2){
                consolePrint("Initiated simulating the system using 'two-step-ahead' receding horizon controller.");
            }
            simulationMode = 6;
            simulationTime = 0;
            discreteTimeSteps = 0;
        }else{ // RHCMethod =3,4,5,6: ED-RHC, ED-RHC-alpha, ED-RHC-Fixed, ED-RHC-Extended
            simulationMode = 7;
            ////simulationTime = 0;
            ////discreteTimeSteps = 0;
        }
        document.getElementById("simulateHybridSystemButton").innerHTML = "<i class='fa fa-pause' aria-hidden='true'></i>";     
        consolePrint("Hybrid system simulation started.");
    }



}


function simulateHybridSystemFast(){ // run the hybrid system for time T period (without displaying agent movements) and get the IPA estimtors

    if(RHCMethod==0){
        consolePrint("Simulated the threshold based controller for a time period T.");
        simulationMode = 2;
    }else if(RHCMethod<3){
        consolePrint("Simulated the greedy receding horizon controller for a time period T.");
        simulationMode = 6;
    }else{
        consolePrint("Simulated the event driven receding horizon controller for a time period T.");
        simulationMode = 7;
    }

    simulationTime = 0;
    discreteTimeSteps = 0;

    dataPlotModeChanged(); // to reset the data arrays


    for(var i = 0; i<targets.length; i++){// rest targets
        targets[i].uncertainty = targets[i].initialUncertainty;
        targets[i].meanUncertainty = targets[i].initialUncertainty;
        
        //// Randomization 4
        targets[i].RHCNoiseR_iEventIndex = 0
        //// end Randomization 4

    }

    for(var i = 0; i<agents.length; i++){// rest agent positions
        agents[i].residingTarget = [agents[i].initialResidingTarget];
        agents[i].position = targets[agents[i].residingTarget[0]].position;
    }
    

    while(simulationTime < periodT){
    
        for(var i = 0; i < targets.length; i++){
            targets[i].updateFastCT(); // update uncertainty levels
        }


        for(var i = 0; i < agents.length; i++){
            if(RHCMethod==0){
                agents[i].updateFastCT(); // update positions of the agents
            }else if (RHCMethod<3){
                agents[i].updateRHCCT();
            }else if (RHCMethod<8){
                agents[i].updateEDRHCCT();
            }else if(RHCMethod<12){
                agents[i].updateEDORHCCT();
            }else if(RHCMethod<13){
                agents[i].updateClassifierCT();
            }else if(RHCMethod<15){
                agents[i].updateRandomCT();
            }else if(RHCMethod==15){
                agents[i].updateEDRHCCT();
            }
        }

        
        simulationTime = simulationTime + deltaT;
        discreteTimeSteps = discreteTimeSteps + 1;
        ////print(simulationTime);


        // Randomization 4: random uncertainty perturbation
        if(RHCNoiseEnabled&&RHCNoiseR_iMagnitude>0){
            for(var i = 0; i < targets.length; i++){
                targets[i].updateRHCNoiseR_i(); // update uncertainty levels
            }
        }
        // Randomization 4: random uncertainty perturbation end


        ////Randomization
        if(dataPlotMode&&RHCNoiseEnabled&&Math.round(discreteTimeSteps)%10==0){recordSystemState();}
        //// end Randomization
    }

    var meanUncertainty = 0;
    for(var i = 0; i<targets.length; i++){
        targets[i].meanUncertainty = (targets[i].meanUncertainty - 0.5*(targets[i].uncertainty + targets[i].initialUncertainty))/discreteTimeSteps;
        meanUncertainty = meanUncertainty + targets[i].meanUncertainty;
    }


    // resetAgentsAndTargets();
    var totalEnergySpent = 0;
    for(var i = 0; i<agents.length; i++){// rest agent positions
        if(agents[i].residingTarget.length==1){
            agents[i].position = targets[agents[i].residingTarget[0]].position;    
        }else if(agents[i].residingTarget.length==2){
            agents[i].position = targets[agents[i].residingTarget[1]].position;
            agents[i].residingTarget = [agents[i].residingTarget[1]];
        }else{

        }
        totalEnergySpent = totalEnergySpent + agents[i].energySpent;
        // agents[i].orientation = 0;
        // agents[i].graphicBaseShapeRotated = agents[i].graphicBaseShape; 

    }
    terminalEnergySpent = totalEnergySpent;
    
    if(RHCMethod<8 | RHCMethod>=12){
        document.getElementById("simulationTime").innerHTML = periodT.toFixed(2).toString();
        document.getElementById("simulationCost").innerHTML = meanUncertainty.toFixed(3).toString();
        consolePrint('Sensing Cost: '+meanUncertainty.toFixed(3).toString());
    }else{
        document.getElementById("simulationTime2").innerHTML = periodT.toFixed(2).toString();
        document.getElementById("simulationCost2").innerHTML = meanUncertainty.toFixed(1).toString();
        document.getElementById("agentEnergyCost").innerHTML = totalEnergySpent.toExponential(2).toString();
        
        terminalTotalCost = meanUncertainty + RHCalpha2*totalEnergySpent;
        document.getElementById("totalCost").innerHTML = terminalTotalCost.toFixed(1).toString();
        consolePrint('Energy Cost: '+totalEnergySpent.toFixed(1).toString()+'; Sensing Cost: '+meanUncertainty.toFixed(3).toString()+'; Total Cost: '+terminalTotalCost.toFixed(1).toString());    
        if(RHCMethod>=8 && RHCMethod<=11){
            var stringListTemp = ['(First-Order-0)','(Second Order)','(First-Order-2)','(First-Order-3)'];
            var stringSelected = stringListTemp[RHCMethod-8];  
            consolePrint('Max agent velocity observed '+stringSelected+': '+(RHCvmaxObserved).toFixed(3).toString())
            consolePrint('Max agent acceleration observed '+stringSelected+': '+(RHCumaxObserved).toFixed(3).toString())
            print([terminalEnergySpent, meanUncertainty, terminalTotalCost, RHCvmaxObserved, RHCumaxObserved])
        }
        
        // if(RHCMethod==9){
        //     // consolePrint('Max agent velocity observed (Second-Order Model): '+(2*RHCvmaxObserved/3).toFixed(3).toString())
        //     consolePrint('Max agent velocity observed (Second-Order Model): '+(RHCvmaxObserved).toFixed(3).toString())
        //     consolePrint('Max agent acceleration observed (Second-Order Model): '+(RHCumaxObserved).toFixed(3).toString())
        //     print([terminalEnergySpent, meanUncertainty, terminalTotalCost, RHCvmaxObserved, RHCumaxObserved])
        
        // }else if(RHCMethod==8){
        //     consolePrint('Max agent velocity used (First-Order Model): '+(RHCvmaxObserved).toFixed(3).toString())
        //     consolePrint('Max agent acceleration observed (First-Order Model): '+(RHCumaxObserved).toFixed(3).toString())
        //     print([terminalEnergySpent, meanUncertainty, terminalTotalCost, RHCvmaxObserved, RHCumaxObserved]);
        // }
        

    }
    

    terminalMeanSystemUncertainty = meanUncertainty;
    //print("Cost : "+meanUncertainty);
    simulationMode = 0;

    if(dataPlotMode){
        plotAdditionalData(meanUncertainty);
    }

}


function recordSystemState(){

    for(var i = 0; i<targets.length; i++){
        targetStateData[i].push(targets[i].uncertainty);
    }
    
    for(var a = 0; a<agents.length; a++){
        agentStateData[a].push(agents[a].residingTarget[0]+1);
        // if(agents[a].residingTarget.length==1){
        //     agentStateData[a].push(agents[a].residingTarget[0]+1);
        // }else{
        //     agentStateData[a].push(NaN);
        // }
    }

    eventTimeData.push(simulationTime);

}





function solveForIPAEstimators(){ // run the hybrid system for time T period (without displaying agent movements) and get the IPA estimtors

    consolePrint("Initiated running the hybrid system for time perion T and getting IPA estimators.");

    ////simulationMode = 3;
    simulationTime = 0;
    discreteTimeSteps = 0;
    eventTime = 0;
    eventCount = 0;


    
    // reset target uncertainties and agent positions
    for(var i = 0; i<targets.length; i++){// rest targets
        targets[i].uncertainty = targets[i].initialUncertainty;
        targets[i].meanUncertainty = targets[i].initialUncertainty;
    }
    for(var i = 0; i<agents.length; i++){// rest agent positions
        agents[i].residingTarget = [agents[i].initialResidingTarget];
        agents[i].position = targets[agents[i].residingTarget[0]].position;

        agents[i].visitedTargetlist = []; // resetting for boosting
        agents[i].unvisitedTargetsInfo = []; // resetting for boosting
    }
    // end reset uncertainties



    // variable initialization

    for(var i = 0; i<targets.length; i++){
        targets[i].residingAgents = [];
        for(var j = 0; j <agents.length; j++){
            targets[i].residingAgents[j] = false;
            if(agents[j].residingTarget[0]==i){
                targets[i].residingAgents[j] = true;
                agents[j].residingTargetUncertainty = targets[i].uncertainty;
            }
        }
    }


    for(var z = 0; z<agents.length; z++){
        agents[z].sensitivityOfThreshold = [];
        for(var p = 0; p < targets.length; p++){
            agents[z].sensitivityOfThreshold[p] = [];
            for(var q = 0; q < targets.length; q++){
                agents[z].sensitivityOfThreshold[p][q] = 0;
            }
        }
    }

    for(var i = 0; i<targets.length; i++){
        targets[i].sensitivityOfUncertainty = [];
        for(var z = 0; z<agents.length; z++){
            targets[i].sensitivityOfUncertainty[z] = []; 
            for(var p = 0; p < targets.length; p++){
                targets[i].sensitivityOfUncertainty[z][p] = [];
                for(var q = 0; q < targets.length; q++){
                    targets[i].sensitivityOfUncertainty[z][p][q] = 0;
                }
            }
        }   
    }


    for(var z = 0; z<agents.length; z++){
        eventTimeSensitivity[z] = []; 
        for(var p = 0; p < targets.length; p++){
            eventTimeSensitivity[z][p] = [];
            for(var q = 0; q < targets.length; q++){
                eventTimeSensitivity[z][p][q] = 0;
            }
        }
    }   
       
    
    // end variable initialization


    



    // HS simulation
    while(simulationTime < periodT){
    
        for(var i = 0; i < targets.length; i++){
            targets[i].updateIPA(); // update uncertainty levels
        }


        for(var i = 0; i < agents.length; i++){
            agents[i].updateIPA(); // update positions of the agents
        }

        simulationTime = simulationTime + deltaT;
        simulationTime = Number(simulationTime.toFixed(3));
        discreteTimeSteps = discreteTimeSteps + 1;
        
    }
    // end HS simulation


    // computing objective function
    var meanUncertainty = 0;
    for(var i = 0; i<targets.length; i++){
        targets[i].meanUncertainty = (targets[i].meanUncertainty - 0.5*(targets[i].uncertainty + targets[i].initialUncertainty))/discreteTimeSteps;
        meanUncertainty = meanUncertainty + targets[i].meanUncertainty;
    }
    // end computing 



    // sensitivity of thresholds
    ////sensitivityUpdateAtEvent(); // consider ending the simulation as an event
    for(var z = 0; z<agents.length; z++){
        for(var p = 0; p < targets.length; p++){
            for(var q = 0; q < targets.length; q++){
                //agents[z].sensitivityOfThreshold[p][q] = agents[z].sensitivityOfThreshold[p][q]/simulationTime;
                if((eventTime-firstEventTime)>0){
                    agents[z].sensitivityOfThreshold[p][q] = agents[z].sensitivityOfThreshold[p][q]/(eventTime-firstEventTime);
                }else{
                    agents[z].sensitivityOfThreshold[p][q] = agents[z].sensitivityOfThreshold[p][q]/simulationTime;
                }
                ////print("p="+p+", q="+q+"; z="+z+".sen^z_pq:"+agents[z].sensitivityOfThreshold[p][q]);
            }
        }
    }
    // display sensitivities as well as get an idea about the general gradient magnitude
    displayThresholdSensitivities(); 
    
    // end - sensitivity of thresholds


    // resetAgentsAndTargets();
    // for(var i = 0; i<agents.length; i++){// rest agent positions
    //     if(agents[i].residingTarget.length==1){
    //         agents[i].position = targets[agents[i].residingTarget[0]].position;    
    //     }else if(agents[i].residingTarget.length==2){
    //         agents[i].position = targets[agents[i].residingTarget[1]].position;
    //         agents[i].residingTarget = [agents[i].residingTarget[1]];
    //     }else{
    //     }
    // }
    // end rest agents and targets

   
    document.getElementById("simulationTime").innerHTML = simulationTime.toFixed(2).toString();
    document.getElementById("simulationCost").innerHTML = meanUncertainty.toFixed(3).toString();

    // tracking the best/worst cost found so far
    if(meanUncertainty<bestCostFoundSoFar && boostingMode==0){
        bestCostFoundSoFar = meanUncertainty;
        
        // cloning multi-dimentional arrays is a tricky thing! - better to do element wise
        bestThresholdsFoundSoFar = [];
        for(var a = 0; a<agents.length; a++){
            bestThresholdsFoundSoFar.push([]);
            for(var i = 0; i<targets.length; i++){//rows
                bestThresholdsFoundSoFar[a].push([]);
                for(var j = 0; j<targets.length; j++){//columns
                    bestThresholdsFoundSoFar[a][i][j] = agents[a].threshold[i][j];
                }
            }
        }
    
    }
    if(meanUncertainty > worstCostFoundSoFar && boostingMode==0){// just for plotting purposes
        worstCostFoundSoFar = meanUncertainty;
    }


    // end tracking best cost so far

    print("Cost : "+meanUncertainty.toFixed(3));

}


function sensitivityUpdateAtEvent(){

    var eventTimePeriod = simulationTime - eventTime;
    eventTime = simulationTime;    
    eventCount = eventCount + 1; 
 
    if(eventCount==1){
        firstEventTime = eventTime;
        if(printMode){print("First event of the simulation occured at: "+eventTime);}
    }

    for(var i = 0; i<targets.length; i++){

        for(var z = 0; z<agents.length; z++){
            for(var p = 0; p < targets.length; p++){
                for(var q = 0; q < targets.length; q++){

                    agents[z].sensitivityOfThreshold[p][q] = agents[z].sensitivityOfThreshold[p][q] + targets[i].sensitivityOfUncertainty[z][p][q]*eventTimePeriod;
                    ////print("p="+p+", q="+q+"; z="+z+".sentheta^z_pq:"+agents[z].sensitivityOfThreshold[p][q]);
                
                }
            }
        }
    }
}




function optimizeThresholdsOneStep(){

    if(simulationMode != 5 && simulationMode !=4){// taking  step for the first time

        simulationMode = 5;
        costArrayForPlot = [];
        boostedCostArrayToPlot = [];
        updateStepCountArray = [];
        numberOfUpdateStepsCount = 0;
        numberOfUpdateStepsCountTemp = 0;
        // to reset from perturbations
        defaultUncertaintyRates = []; // to test boosting
        defaultSensingRates = []; // to test boosting
        for(var i = 0; i<targets.length; i++){
            defaultUncertaintyRates.push(targets[i].uncertaintyRate);
        }
        for(var i = 0; i<agents.length; i++){
            defaultSensingRates.push(agents[i].sensingRate);
        }
        // to reset from perturbations
        consolePrint("Finding optimal threshold policy using IPA estimators started (Step-by-Step).");
    
    }

    simulationMode = 5;
    solveForIPAEstimators();
    updateThresholdPolicy(); 
    displayThresholds();
    numberOfUpdateStepsCount = numberOfUpdateStepsCount + 1;
    numberOfUpdateStepsCountTemp = numberOfUpdateStepsCountTemp + 1;
    var cost = Number(document.getElementById("simulationCost").innerHTML);
    updateStepCountArray.push(numberOfUpdateStepsCount);

    var justSwitched = numberOfUpdateStepsCount>1 && numberOfUpdateStepsCountTemp == 1;

    if(boostingMode==0 && !justSwitched){
        costArrayForPlot.push(cost);
        consolePrint("Iteration "+ numberOfUpdateStepsCount+ " completed. Cost: "+cost+".");
        boostedCostArrayToPlot.push(NaN);
    }else{// in boosting mode!
        var mappedCost = cost;
        // if(cost>worstCostFoundSoFar){ // upper bound threshold so that plot wont be distorted
        //     mappedCost = worstCostFoundSoFar;
        // }else if(cost<bestCostFoundSoFar){
        //     mappedCost = bestCostFoundSoFar;
        // }
        boostedCostArrayToPlot.push(mappedCost);
        consolePrint("Iteration "+ numberOfUpdateStepsCount+ " completed (while in boosting mode). Cost: "+cost+".");                
        
        costArrayForPlot.push(NaN);
    }
    plotData();

    ////consolePrint("IPA step "+ numberOfUpdateStepsCount+ " completed. Cost: "+cost+".");
    
}



function optimizeThresholds(){// use solveForIPAEstimators() iteratively to updte the thresholds 

    consolePrint("Finding optimal threshold policy using IPA estimators started.");

    if(simulationMode == 5 || simulationMode == 4){// have been using the step-by-step method, so just continue
        simulationMode = 4;
    }else{
        simulationMode = 4;
        costArrayForPlot = [];
        boostedCostArrayToPlot = [];
        updateStepCountArray = [];
        numberOfUpdateStepsCount = 0;
        numberOfUpdateStepsCountTemp = 0;

        // to reset from perturbations
        defaultUncertaintyRates = []; // to test boosting
        defaultSensingRates = []; // to test boosting
        for(var i = 0; i<targets.length; i++){
            defaultUncertaintyRates.push(targets[i].uncertaintyRate);
        }
        for(var i = 0; i<agents.length; i++){
            defaultSensingRates.push(agents[i].sensingRate);
        }
        // to reset from perturbations
    }

    // test case:
    // agents[0].threshold = [[16.3397,5.3083,5.1771,1.7396,0.7245],[2.8752,1.0177,18.5617,22.1334,24.5507],[23.7571,9.9299,9.8039,23.8161,8.4909],[12.0548,5.8283,4.5564,23.2786,17.6672],[21.8119,21.0435,18.5885,10.3858,9.0469]];
    // agents[1].threshold = [[0.8776,22.1273, 3.3326,10.8125,22.2834],[21.3765,22.6049,17.4481, 0.4536,22.9551],[16.4313, 0.2605, 9.9551,17.2859, 1.8280],[19.1421, 1.8639,22.0781,11.7423, 1.1362],[13.8522, 6.1157, 4.5252, 3.2056,10.9643]];
    // displayThresholds();
}

function updateThresholdPolicy(){


    // stepSize is a global variable
    if(stepSizeSelectionMethod==0){//constant step
        stepSizeValue = stepSize;
    }else if(stepSizeSelectionMethod==1){// diminishing
        stepSizeValue = stepSize/Math.sqrt(numberOfUpdateStepsCountTemp+1);
    }else if(stepSizeSelectionMethod==2){
        stepSizeValue = stepSize/(stepSize + numberOfUpdateStepsCountTemp+1);
    }else{
        stepSizeValue = stepSize;
    }
    // end step size selection
    


    thresholdSensitivityMagnitudes = [];
    var globalSensitivityMagnitude = 0;

    for(var z = 0; z<agents.length; z++){
        var maxRowSum = 0;
        for(var p = 0; p < targets.length; p++){
            var rowAbsoluteSum = 0;
            for(var q = 0; q < targets.length; q++){
                var val = agents[z].threshold[p][q] - stepSizeValue*agents[z].sensitivityOfThreshold[p][q];
                if(val<=0){
                    val = 0;
                }else if(val>10000 && val<1000000){
                    val = 10000;
                } 

                if(val<=10000){
                    rowAbsoluteSum = rowAbsoluteSum + Math.abs(agents[z].threshold[p][q] - val);
    
                    agents[z].threshold[p][q] = val;
                }
            }

            if(rowAbsoluteSum>maxRowSum){
                maxRowSum = rowAbsoluteSum;
            }
        }

        thresholdSensitivityMagnitudes.push(maxRowSum);
        globalSensitivityMagnitude = globalSensitivityMagnitude + maxRowSum;

    }
    

    consolePrint("Global sensitivity magnitude: "+globalSensitivityMagnitude.toFixed(3)+".");
    

    // mode switch
    if(boostingMethod==1 && boostingMode==0 && globalSensitivityMagnitude<modeSwitchingThresholdAlpha){
        initiateForcedModeSwitch();
    }else if(boostingMethod==1 && boostingMode==1 && globalSensitivityMagnitude<10*modeSwitchingThresholdAlpha){
        initiateForcedModeSwitch();
    }

    if(boostingMethod==2 && boostingMode==0 && globalSensitivityMagnitude<modeSwitchingThresholdAlpha){
        initiateForcedModeSwitch();
    }else if(boostingMethod==2 && boostingMode==1 && (globalSensitivityMagnitude<10*modeSwitchingThresholdAlpha || numberOfUpdateStepsCountTemp>10)){
        initiateForcedModeSwitch();
    }

    if(boostingMethod==3 && boostingMode==0 && globalSensitivityMagnitude<modeSwitchingThresholdAlpha){
        addRandomNoiseToThresholds();
        initiateForcedModeSwitch();
    }else if(boostingMethod==3 && boostingMode==1 && numberOfUpdateStepsCountTemp == 1 ){
        initiateForcedModeSwitch();
        //addRandomNoiseToThresholds();
    }

    if(boostingMethod==4 && boostingMode==0 && globalSensitivityMagnitude<modeSwitchingThresholdAlpha){
        initiateForcedModeSwitch();
    }else if(boostingMethod==4 && boostingMode==1 && globalSensitivityMagnitude<modeSwitchingThresholdAlpha){
        initiateForcedModeSwitch();
        //addRandomNoiseToThresholds();
    }

    if(boostingMethod==5 && boostingMode==0 && globalSensitivityMagnitude<modeSwitchingThresholdAlpha){
        initiateForcedModeSwitch();
    }else if(boostingMethod==5 && boostingMode==1 && globalSensitivityMagnitude<modeSwitchingThresholdAlpha){
        initiateForcedModeSwitch();
        //addRandomNoiseToThresholds();
    }




}


function displayThresholds(){

    for(var a = 0; a<agents.length; a++){

        for(var i = 0; i<targets.length; i++){//rows
            
            for(var j = 0; j<targets.length; j++){//columns
                
                var threshold = agents[a].threshold[i][j];
                document.getElementById("customRTThreshold_"+(i+1)+"_"+(j+1)+"_"+(a+1)).innerHTML = threshold.toFixed(3);

            }
            
        }

    }


}

function resetAgentsAndTargets(){

    for(var i = 0; i<targets.length; i++){// rest targets
        targets[i].uncertainty = targets[i].initialUncertainty;
        targets[i].meanUncertainty = targets[i].initialUncertainty;
    }

    for(var i = 0; i<agents.length; i++){// rest agent positions
        if(agents[i].residingTarget.length==1){
            agents[i].position = targets[agents[i].residingTarget[0]].position;    
        }else if(agents[i].residingTarget.length==2){
            agents[i].position = targets[agents[i].residingTarget[1]].position;
            agents[i].residingTarget = [agents[i].residingTarget[1]];
        }else{

        }
        
    }


}


function resetSimulation(){

    simulationMode = 0;
    simulationTime = 0;
    eventCount = 0;
    eventTime = 0;
    discreteTimeSteps = 0;

    
    for(var i = 0; i<targets.length; i++){// rest targets
        targets[i].uncertainty = targets[i].initialUncertainty;
        targets[i].meanUncertainty = targets[i].initialUncertainty;
        
        //// Randomization 4
        targets[i].RHCNoiseR_iEventIndex = 0
        //// end Randomization 4

        //// Randomization 3
        targets[i].position = targets[i].initialPosition
        targets[i].oldPosition = targets[i].initialPosition
        //// end Randomization 3
    }

    for(var i = 0; i<agents.length; i++){// rest agent positions
        agents[i].residingTarget = [agents[i].initialResidingTarget];
        agents[i].position = targets[agents[i].residingTarget[0]].position;
        
        agents[i].timeToExitMode = [3,0]; // Event Driven RHC
        agents[i].energySpent = 0;
    }

    if(RHCMethod<8 | RHCMethod>=12){
        document.getElementById("simulationTime").innerHTML = simulationTime.toFixed(2).toString();
        document.getElementById("simulationCost").innerHTML = 0;
    }else{
        RHCumaxObserved = 0;
        RHCvmaxObserved = 0;

        document.getElementById("simulationTime2").innerHTML = simulationTime.toFixed(2).toString();
        document.getElementById("simulationCost2").innerHTML = 0;
        document.getElementById("agentEnergyCost").innerHTML = 0;
        document.getElementById("totalCost").innerHTML = 0;
    }

    document.getElementById("simulateHybridSystemButton").innerHTML = "<i class='fa fa-play' aria-hidden='true'></i>"; 
    consolePrint("Simulation stopped and reseted to the initial state.");
}







function pauseSimulation(){

    if(RGCComputingMode>0){
        RGCComputingMode = 0;
        simulationMode = 1;
    }

    if(simulationMode == 0){
        simulationMode = 1;
        document.getElementById("pauseButton").innerHTML = "<i class='fa fa-pause' aria-hidden='true'></i>";     
        consolePrint("Hybrid system simulation started.");
    }else{
        simulationMode = 0;
        document.getElementById("pauseButton").innerHTML = "<i class='fa fa-play' aria-hidden='true'></i>"; 
        consolePrint("Hybrid system simulation paused.");
    }



    // // start simulation button
    // if(RHCMethod==0){//disabled
    //     // old pause button code
    //     if(RGCComputingMode>0){
    //         RGCComputingMode = 0;
    //         simulationMode = 1;
    //     }

    //     if(simulationMode == 0){
    //         simulationMode = 1;
    //         document.getElementById("pauseButton").innerHTML = "<i class='fa fa-pause' aria-hidden='true'></i>";     
    //         consolePrint("Hybrid system simulation started.");
    //     }else{
    //         simulationMode = 0;
    //         document.getElementById("pauseButton").innerHTML = "<i class='fa fa-play' aria-hidden='true'></i>"; 
    //         consolePrint("Hybrid system simulation paused.");
    //     }  
    // }
    // else if(RHCMethod<3){
    //     if(RHCMethod==1){
    //         consolePrint("Initiated simulating the system using 'one-step-ahead' receding horizon controller.");
    //     }else if(RHCMethod==2){
    //         consolePrint("Initiated simulating the system using 'two-step-ahead' receding horizon controller.");
    //     }
    //     simulationMode = 6;
    //     //////simulationTime = 0;
    //     //////discreteTimeSteps = 0;
    // }else{ // RHCMethod =3
    //     simulationMode = 7;
    //     //////simulationTime = 0;
    //     //////discreteTimeSteps = 0;
    // }






    
}

function stopSimulation(){
    
    if(simulationMode == 4){// continuous running IPA to step mode
        simulationMode = 5;
    }else{
        simulationMode = 0;    
    }
    
    simulationTime = 0;
    discreteTimeSteps = 0;

    resetAgentsAndTargets();

    document.getElementById("simulateHybridSystemButton").innerHTML = "<i class='fa fa-play' aria-hidden='true'></i>"; 

    consolePrint("Simulation stopped and reseted to the initial state.");

}

function similarityMeasureTypeChanged(){
    similarityMeasureType = Number(document.getElementById('similarityMeasureTypeDropdown').value);
    if(similarityMeasureType==0){
        consolePrint('Similarity measure type changed. Now, similarity between two targets are defined by:');
        consolePrint('Length of the shortest path between the two targets.');
    }else{
        consolePrint('Similarity measure type changed. Now, similarity between two targets are defined by:');
        consolePrint('Minimum (over all possible cycles) mean cycle uncertainty of a cycle containing both targets.');
    }
    adjustNeighborhoodWidthForClusteringRange();
}

function neighborhoodWidthForClusteringChanged(value){
    neighborhoodWidthForClustering = value;
    document.getElementById('neighborhoodWidthForClusteringDisplay').innerHTML = neighborhoodWidthForClustering.toString();
    consolePrint('Neighborhood width used for clustering (to transform the similarity measure) changed!');
}

function spectralClusteringMethodChanged(){
    spectralClusteringMethod = Number(document.getElementById('spectralClusteringMethodDropdown').value);
    if(spectralClusteringMethod==0){
        consolePrint('Spectral clustering method changed to: Unnormalized spectral clustering.');    
    }else if(spectralClusteringMethod==1){
        consolePrint('Spectral clustering method changed to: Normalized spectral clustering proposed in [Shi and Malik 2000].');
    }else if(spectralClusteringMethod==2){
        consolePrint('Spectral clustering method changed to: Normalized spectral clustering proposed in [Ng, Jordan, and Weiss 2002].');
    }
}


function adjustNeighborhoodWidthForClusteringRange(){

    var similarityMatrix = computeSimilarityMatrix();
    print(similarityMatrix);
    var M = targets.length;
    var maxSimilarity = 0;
    var minSimilarity = 0;

    for(var i = 0; i < M; i++){
        for(var j = 0; j < M; j++){
            if(similarityMatrix[i][j]>maxSimilarity && similarityMatrix[i][j]<Infinity){
                maxSimilarity = similarityMatrix[i][j]
            }else if(similarityMatrix[i][j]>0 && similarityMatrix[i][j]<minSimilarity){
                minSimilarity = similarityMatrix[i][j];
            }
        }
    }

    var maxSigma = maxSimilarity/3;
    var minSigma = minSimilarity/3;
    var defaultSigma = (0.7*minSigma+0.3*maxSigma); // maxSimilarity is beyond 3*sigma
    var stepSizeOfTheSlider = (maxSigma-minSigma)/100;
    
    document.getElementById('neighborhoodWidthForClustering').min = minSigma.toFixed(3);
    document.getElementById('neighborhoodWidthForClustering').max = maxSigma.toFixed(3);
    document.getElementById('neighborhoodWidthForClustering').step = stepSizeOfTheSlider.toFixed(3);
    document.getElementById('neighborhoodWidthForClustering').value = defaultSigma.toFixed(3);
    
    neighborhoodWidthForClusteringChanged(defaultSigma.toFixed(3));
    consolePrint("Neighborhood width selecting slider's parameters are tuned." );
}

function resetGraphClusters(){
    // this function should have the inverse effect of the function
    // applyFoundClustersToGroupTargets(clusteredPoints) found in mathtools.js 

    targetClusters = [];
    interClusterPaths = []; 
    for(var p = 0; p < paths.length; p++){// check all paths
        paths[p].brokenDueToClustering = false;
    }
    resetCycles();
    displayClustersMode = false;

}

function refreshRandomProblemConfiguration(nT,nA,maxLinkmLength){
    removeAll();
    startModifyingProbConfig();
    for(var i = 0; i<nT; i++){
        addATargetAt(500*Math.random(),500*Math.random());
    }
    disconnectAllPaths();
    document.getElementById('maximumPathLength').value = maxLinkmLength;
    maximumPathLengthChanged();
    addAnAgentAtTarget(0);
    for(var k=1;k<nA;k++){
        addAnAgentAtTarget(k*Math.round(nT/nA));
    }
    finishModifyingProbConfig();
    dataPlotModeChanged();
}

function numOfKMeansIterationsChanged(value){
    numberOfKMeansIterations = value;
    consolePrint("Number of K-Means steps (realizations) changed to "+value+".")
}

function frameRateChanged(value){
    simulationFrameRate = value;
    document.getElementById("frameRateDisplay").innerHTML = simulationFrameRate.toString();
    consolePrint("Visualization frame rate changed to "+simulationFrameRate+" frames per &Delta;t.");    
}

function numberOfUpdateStepsChanged(value){
    numberOfUpdateSteps = value;
    consolePrint("Total number of gradient update steps changed to "+numberOfUpdateSteps+".");
    
}



function problemConfigurationChanged(){
    
    var r = document.getElementById("arrivalDistributionDropdown").value;
    
    
    if(r == 9){
        removeAll();
        startModifyingProbConfig();
        for(var i = 0; i<15; i++){
            addATargetAt(500*Math.random(),500*Math.random());
        }
        disconnectAllPaths();
        document.getElementById('maximumPathLength').value = 200;
        maximumPathLengthChanged();
        addAnAgentAtTarget(0);
        addAnAgentAtTarget(5);
        addAnAgentAtTarget(10);
        finishModifyingProbConfig();

    }else if(r == 8){
        removeAll();
        startModifyingProbConfig();
        addATargetAt(135,486);
        addATargetAt(115,375);
        addATargetAt(310,485);
        addATargetAt(297,282);
        addATargetAt(106,176);
        addATargetAt(190,105);
        addATargetAt(405,104);
        addATargetAt(494,164);
        addATargetAt(485,354);
        addATargetAt(476,484);
        disconnectAllPaths();
        connectPathsBetween([0,1,0,2,1,2,1,3,1,4,2,3,2,9,2,8]);
        connectPathsBetween([3,4,3,5,3,6,3,7,3,8,4,5,5,6,6,7,7,8,8,9]);
        addAnAgentAtTarget(0);
        addAnAgentAtTarget(4);
        addAnAgentAtTarget(8);
        finishModifyingProbConfig();

    }else if(r==7.5){
        removeAll();
        startModifyingProbConfig();
        addATargetAt(110,50);
        addATargetAt(530,110);
        addATargetAt(480,510);
        addATargetAt(60,470);
        addATargetAt(250,220);
        addATargetAt(350,250);
        addATargetAt(320,350);
        addATargetAt(220,320);
        disconnectAllPaths();
        connectPathsBetween([0,1,1,2,2,3,0,3,0,4,1,5,2,6,3,7,4,5,5,6,6,7,7,4,0,7,1,4,2,5,3,6,4,6,5,7,4,7]);
        addAnAgentAtTarget(0);
        finishModifyingProbConfig();

    }else if(r==7){
        removeAll();
        startModifyingProbConfig();
        addATargetAt(55,405);
        addATargetAt(58,107);
        addATargetAt(153,268);
        addATargetAt(191,435);
        addATargetAt(189,532);
        addATargetAt(277,234);
        addATargetAt(403,315);
        addATargetAt(372,485);
        addATargetAt(486,224);
        addATargetAt(521,442);
        disconnectAllPaths();
        connectPathsBetween([0,1,1,2,2,3,3,4,2,5,3,5,5,6,5,8,5,7,6,7,6,8,8,9]);
        addAnAgentAtTarget(0);
        addAnAgentAtTarget(4);
        addAnAgentAtTarget(8);
        finishModifyingProbConfig();

    }else if(r==6){// Maze with 10 agents - Iterative greedy 200 descretized points (adjusted-CoverageV34)
        removeAll();
        startModifyingProbConfig();
        addATargetAt(63,488);
        addATargetAt(67,288);
        addATargetAt(120,100);
        addATargetAt(234,105);
        addATargetAt(267,317);
        addATargetAt(297,491);
        addATargetAt(433,505);
        addATargetAt(534,402);
        addATargetAt(465,263);
        addATargetAt(446,58);
        disconnectAllPaths();
        connectPathsBetween([0,1,1,2,2,3,3,9,3,4,4,5,4,8,5,6,6,7,7,8,7,8,8,9]);
        addAnAgentAtTarget(0);
        addAnAgentAtTarget(4);
        addAnAgentAtTarget(8);
        finishModifyingProbConfig();

    }else if(r==5){// Maze with 10 agents - Iterative greedy 200 descretized points (adjusted-CoverageV34)
        removeAll();
        startModifyingProbConfig();
        addATargetAt(53,185);
        addATargetAt(98,504);
        addATargetAt(318,530);
        addATargetAt(529,485);
        addATargetAt(552,282);
        addATargetAt(495,67);
        addATargetAt(188,105);
        addATargetAt(202,380);
        addATargetAt(400,405);
        addATargetAt(353,193);
        disconnectAllPaths();
        connectPathsBetween([0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9]);
        addAnAgentAtTarget(0);
        addAnAgentAtTarget(4);
        addAnAgentAtTarget(8);
        finishModifyingProbConfig();
    }else if(r==4){
        removeAll();
        startModifyingProbConfig();
        addATargetAt(50,50);
        addATargetAt(50,250);
        addATargetAt(50,450);
        addATargetAt(250,450);
        addATargetAt(450,450);
        addATargetAt(450,250);
        addATargetAt(450,50);
        addATargetAt(250,50);
        addATargetAt(250,250);
        disconnectAllPaths();
        connectPathsBetween([0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8]);
        addAnAgentAtTarget(0);
        addAnAgentAtTarget(4);
        addAnAgentAtTarget(8);
        finishModifyingProbConfig();
    }else if(r==3.75){
        removeAll();
        startModifyingProbConfig();
        addATargetAt(40,370);
        addATargetAt(130,480);
        addATargetAt(240,540);
        addATargetAt(400,480);
        addATargetAt(500,370);
        addATargetAt(470,210);
        addATargetAt(340,100);
        addATargetAt(160,80);
        addATargetAt(60,140);
        disconnectAllPaths();
        document.getElementById('maximumPathLength').value = 240;
        maximumPathLengthChanged();
        addAnAgentAtTarget(0);
        addAnAgentAtTarget(3);
        addAnAgentAtTarget(6);
        finishModifyingProbConfig();
    }else if(r==3.5){
        removeAll();
        startModifyingProbConfig();
        addATargetAt(40,370);
        addATargetAt(130,480);
        addATargetAt(240,540);
        addATargetAt(360,580);
        addATargetAt(500,370);
        addATargetAt(470,210);
        addATargetAt(340,100);
        addATargetAt(160,80);
        addATargetAt(60,140);
        disconnectAllPaths();
        document.getElementById('maximumPathLength').value = 460;
        maximumPathLengthChanged();
        addAnAgentAtTarget(0);
        finishModifyingProbConfig();
    }else if(r==3){ // 4 targets 1 agent
        removeAll();
        startModifyingProbConfig();
        addATargetAt(50,450);
        addATargetAt(250,450);
        addATargetAt(250,250);
        addATargetAt(50,250);
        paths[1].isPermenent = false;
        paths[4].isPermenent = false;
        addAnAgentAtTarget(0);
        finishModifyingProbConfig();
    }else if(r==2.5){//1A arrangement
        removeAll();
        startModifyingProbConfig();
        addATargetAt(50,450);
        addATargetAt(50,300);
        addATargetAt(550,450);
        addATargetAt(300,100);
        addATargetAt(150,300);
        paths[2].isPermenent = false;
        paths[3].isPermenent = false;
        addAnAgentAtTarget(0);
        finishModifyingProbConfig();
    }else if(r==2){//2A arrangement
        removeAll();
        startModifyingProbConfig();
        addATargetAt(50,450);
        addATargetAt(50,300);
        addATargetAt(550,450);
        addATargetAt(300,100);
        addATargetAt(150,300);
        paths[2].isPermenent = false;
        paths[3].isPermenent = false;
        addAnAgentAtTarget(0);
        addAnAgentAtTarget(2);
        finishModifyingProbConfig();
    }else if(r==1){
        removeAll();
        startModifyingProbConfig();
        addATargetAt(50,450);
        addATargetAt(50,300);
        addATargetAt(250,450);
        addAnAgentAtTarget(0);
        finishModifyingProbConfig();
    }else if(r==0){
        removeAll();
        startModifyingProbConfig();
        document.getElementById("spaceForRealTimeUncertaintyRateEdit").style.display = "none";
        document.getElementById("spaceForRealTimeSensingRateEdit").style.display = "none";
        document.getElementById("spaceForRealTimeUncertaintyEdit").style.display = "none";
        document.getElementById("spaceForRealTimeThresholdEdit").style.display = "none";
        document.getElementById("spaceForThresholdSensitivities").style.display = "none";
        
        document.getElementById("runSimulationWizard").style.display = "none";

    }
    // print("Num of targets: "+targets.length)
    dataPlotModeChanged();

    
}


function removeAll(){
    startModifyingProbConfig();
    for(var i = agents.length; i > 0; i--){
        removeAnAgent();
    }
    for(var i = targets.length; i > 0; i--){
        removeATarget();
    }
    for(var i = cycles.length; i > 0; i--){
        removeACycle();
    }

    finishModifyingProbConfig();
}

function sleepFor(miliseconds) {
   var currentTime = new Date().getTime();

   while (currentTime + miliseconds >= new Date().getTime()) {
   }
}


function fullyConnectCBChanged(val){
    if(val){
        consolePrint("Making the graph fully connected.");
        for(var i = 0; i<paths.length; i++){
            if(!paths[i].isPermenent){
                paths[i].artificiallyExtended = true;
                paths[i].isPermenent = true;
            }
        }
    }else{
        consolePrint("Resetting the fully connected graph.");
        for(var i = 0; i<paths.length; i++){
            if(paths[i].artificiallyExtended){
                 paths[i].isPermenent = false;
                 paths[i].artificiallyExtended = false;
            }
        }
    }
}



function generateCostVsHorizonCurve(resVal){
    var minCostTh = 1;
    var minCost = Infinity;
    var cost = "";

    var T_hNow = Number(document.getElementById("timeHorizonForRHC").value);
    var res = T_hNow/resVal;
    var T_hLow = 1;
    var T_hHigh = 1.2*T_hNow;//1.5*T_hNow;

    for(var t_h=T_hLow; t_h<T_hHigh; t_h=t_h+res){
        resetSimulation();
        document.getElementById("timeHorizonForRHC").value = t_h;
        timeHorizonForRHCChanged(t_h);
        simulateHybridSystemFast();
        cost = cost+t_h.toFixed(3)+","+terminalMeanSystemUncertainty.toFixed(3)+";";

        if(terminalMeanSystemUncertainty<minCost){
            minCostTh = t_h;
            minCost = terminalMeanSystemUncertainty;
        }

    }

    resetSimulation();
    document.getElementById("timeHorizonForRHC").value = minCostTh;
    timeHorizonForRHCChanged(minCostTh);
    simulateHybridSystemFast();

    consolePrint("Best T_h = "+minCostTh.toFixed(3)+", which gives J = "+minCost.toFixed(3)+".")
    print(cost);

}

function plotCostVsParameter(){

    var paraType = Number(document.getElementById("plotCostVsParameterDropDown").value)
    var startingValue;
    if(paraType==2||paraType==3){
        document.getElementById("RHCParamterOverrideCB").checked = true;
        RHCParametersChanged();
    }else if(paraType==0){
        startingValue = periodT;
    }else if(paraType==5){// v_max
        document.getElementById("RHCParamterFixAlphaCB").checked = true;
        RHCParametersChanged();
    }else if(paraType==6){// alpha
        document.getElementById("RHCParamterFixAlphaCB").checked = false;
        RHCParametersChanged();
    }

    var startVal = Number(document.getElementById("plotCostVsParameterStart").value);
    var resVal = Number(document.getElementById("plotCostVsParameterRes").value);
    var endVal = Number(document.getElementById("plotCostVsParameterEnd").value);

    var minCostPara = startVal;
    var minCost = Infinity;
    var costArray;
    var paraArray;
    if(paraType==5 || paraType==6){
        costArray = [[],[],[],[],[]] // alpha*J_e, J_s, J_T, v_max, u_max
        paraArray = [[],[]]; //beta, alpha
    }else{
        costArray = [];
        paraArray = [];
    }

    var oldDataPlotMode = document.getElementById("dataPlotModeCheckBox").checked;
    document.getElementById("dataPlotModeCheckBox").checked = false; // to speed up 
    dataPlotModeChanged();
    
    for(var para=startVal; para<=endVal; para=para+resVal){
        resetSimulation();
        
        if(paraType==0){// T changed
            
            document.getElementById("periodT").value = para;
            periodTChanged(para);

        }else if(paraType==1){// H changed
            
            document.getElementById("timeHorizonForRHC").value = para;
            timeHorizonForRHCChanged(para);
        
        }else if(paraType==2){// alpha changed

            document.getElementById("RHCalpha").value = para;
            RHCParametersChanged();

        }else if(paraType==3){// beta changed

            document.getElementById("RHCbeta").value = para;
            RHCParametersChanged();

        }else if(paraType==5){// v_max changed

            document.getElementById("RHCvmax").value = para;
            RHCParametersChanged();

        }else if(paraType==6){// \beta in alpha_u 

            document.getElementById("RHCalpha2").value = para;
            RHCParametersChanged();

        }
        
        simulateHybridSystemFast();
        
        if(paraType<5){
            costArray.push(terminalMeanSystemUncertainty);
            paraArray.push(para);

            if(terminalMeanSystemUncertainty<minCost){
                minCostPara = para;
                minCost = terminalMeanSystemUncertainty;
            }
        }else if(paraType==5||paraType==6){
            costArray[0].push(terminalEnergySpent*RHCalpha2)
            costArray[1].push(terminalMeanSystemUncertainty)
            costArray[2].push(terminalTotalCost)
            costArray[3].push(RHCvmaxObserved)
            costArray[4].push(RHCumaxObserved)


            paraArray[0].push(para);// v_max or beta
            paraArray[1].push(RHCalpha2);// alpha2

            if(terminalTotalCost<minCost){
                minCostPara = para;
                minCost = terminalTotalCost;
            }

        }

    }


    // getting things back to normal
    resetSimulation();
    
    if(paraType==0){// T changed
            
        document.getElementById("periodT").value = startingValue;
        periodTChanged(startingValue);

    }else if(paraType==1){// H changed
        
        document.getElementById("timeHorizonForRHC").value = minCostPara;
        timeHorizonForRHCChanged(minCostPara);
    
    }else if(paraType==2){// alpha changed

        document.getElementById("RHCalpha").value = minCostPara;
        RHCParametersChanged();

    }else if(paraType==3){// beta changed

        document.getElementById("RHCbeta").value = minCostPara;
        RHCParametersChanged();

    }else if(paraType==4){// realizations executed

        var sumCost = 0;
        for(var i=0;i<costArray.length;i++){
            sumCost = sumCost + costArray[i];
        }
        var meanCost = sumCost/endVal;

        var variance = 0;
        for(var i=0;i<costArray.length;i++){
            variance = variance + sq(costArray[i]-meanCost);
        }
        variance = variance/endVal;

    }else if(paraType == 5){// v_max changed
        document.getElementById("RHCvmax").value = minCostPara;
        RHCParametersChanged();
    }else if(paraType == 6){// alpha_u changed
        document.getElementById("RHCalpha2").value = minCostPara;
        RHCParametersChanged();
    } 
    
    simulateHybridSystemFast();

    document.getElementById("dataPlotModeCheckBox").checked = oldDataPlotMode; 
    dataPlotModeChanged();
    
    print("Cost Array:")
    print(costArray)

    print("Parameter Array:")
    print(paraArray)
    
    if(paraType!=4){
        consolePrint("Best parameter value = "+minCostPara.toFixed(3)+", which gives J = "+minCost.toFixed(3)+".")
        print("Best para = "+minCostPara.toFixed(3)+", which gives J = "+minCost.toFixed(3)+".")
    }else{
        consolePrint("Cost: Mean= "+meanCost.toFixed(3)+", Variance= "+variance.toFixed(3)+", over "+endVal+" realizations.")
        print("Mean cost: "+meanCost.toFixed(3)+", Variance: "+variance.toFixed(3)+".");
        print(meanCost.toFixed(3)+", "+variance.toFixed(3));
    }

    plotCostVsParameterData(paraType,paraArray,costArray)

}


// function runRandomTests(){

// }


function runARandomTest(repeatedRandomTestMode){

    var cost = "";
       

    document.getElementById("RHCMethodDropdown").value = 3;
    RHCMethodChanged();
    simulateHybridSystemFast();
    cost = cost+terminalMeanSystemUncertainty.toFixed(3);
    resetSimulation();

    document.getElementById("RHCMethodDropdown").value = 4;
    RHCMethodChanged();
    simulateHybridSystemFast();
    cost = cost+","+terminalMeanSystemUncertainty.toFixed(3);
    resetSimulation();

    document.getElementById("RHCMethodDropdown").value = 6;
    RHCMethodChanged();
    simulateHybridSystemFast();
    cost = cost+","+terminalMeanSystemUncertainty.toFixed(3);
    resetSimulation();

    document.getElementById("RHCMethodDropdown").value = 7;
    RHCMethodChanged();
    simulateHybridSystemFast();
    cost = cost+","+terminalMeanSystemUncertainty.toFixed(3)+";";
    resetSimulation();

    return cost

}


function doSave(value, type, name) {
  var blob;
  if (typeof window.Blob == "function") {
    blob = new Blob([value], {
      type: type
    });
  } else {
    var BlobBuilder = window.BlobBuilder || window.MozBlobBuilder || window.WebKitBlobBuilder || window.MSBlobBuilder;
    var bb = new BlobBuilder();
    bb.append(value);
    blob = bb.getBlob(type);
  }
  var URL = window.URL || window.webkitURL;
  var bloburl = URL.createObjectURL(blob);
  var anchor = document.createElement("a");
  if ('download' in anchor) {
    anchor.style.visibility = "hidden";
    anchor.href = bloburl;
    anchor.download = name;
    document.body.appendChild(anchor);
    var evt = document.createEvent("MouseEvents");
    evt.initEvent("click", true, true);
    anchor.dispatchEvent(evt);
    document.body.removeChild(anchor);
  } else if (navigator.msSaveBlob) {
    navigator.msSaveBlob(blob, name);
  } else {
    location.href = bloburl;
  }
}



function plotDecisionBoundary(){
    var a = 0; // agent
    var i = 0; // current target
    targets[i].uncertainty = 0;
    var X = 1; // neighbor targets
    var Y = 2; // neighbor targets
    var res = 0.1;
    var lim = 10;
    var stringVal = "[";
    for(var x = 0; x<=lim; x=x+res){
        for(var y = 0; y<=lim; y=y+res){
            targets[X].uncertainty = x;
            targets[Y].uncertainty = y;
            var ans1 = agents[a].solveOP3(i); 
            var ans2 = agents[a].solveForTheNextVisit(i);

            // print("x = "+x+"; y="+y+"; j="+ans[0]);        
            stringVal = stringVal + x+","+y+","+ans1[0]+","+ans2[0]+";"
        }
    }
    stringVal = stringVal.slice(0, -1);
    stringVal = stringVal+"]";
    return stringVal;
}







function tuneRLBasedEDRHC(numOfIterations){
    // var numOfIterations = 100;
    RLParametersChanged();
    var numOfEpisodesPerIteration = RLNumOfEpisodesPerIteration;

    var costArray = [];
    var paraArray = [];
    var bestCost = Infinity;
    var bestPolicy = storeRestoreActionValueDataMeans(0,0);
    var meanVals = [...bestPolicy];
    for(k = 0; k<numOfIterations; k++){

        // run several episodes and collect data
        var meanEpisodeCost = 0;
        for(l = 0; l<numOfEpisodesPerIteration; l++){
            simulateHybridSystemFast();
            meanEpisodeCost = meanEpisodeCost + (terminalMeanSystemUncertainty - meanEpisodeCost)/(l+1);        
            updateActionValueData(0); // do not update the policy, just update the means
            resetSimulation();
            numOfRLIterations++;
        }
        numOfRLIterations = numOfRLIterations-numOfEpisodesPerIteration;//rollback

        print("Cost at k="+k+", is :"+meanEpisodeCost.toFixed(3));
        costArray.push(meanEpisodeCost);
        paraArray.push(k);
        if(meanEpisodeCost<bestCost){
            bestCost = meanEpisodeCost;
            bestPolicy = [...meanVals];
            //print(bestPolicy[0][0][1])
        }

        updateActionValueData(1);
        meanVals = storeRestoreActionValueDataMeans(0,0);  //stremode,dummy  
        storeRestoreActionValueDataMeans(1,meanVals); // removes all the reward data and episode mean data, keeping the action values
        
        resetSimulation();
        // print(agents[0].actionValueData[0][1])
        numOfRLIterations++;    
    }
    
    plotCostVsParameterData(7,paraArray,costArray)
    
    if(bestCost<10000000000){
        print("Best Cost = "+bestCost.toFixed(3));
        var improvement = 100*(costArray[0]-bestCost)/costArray[0];
        consolePrint("A control policy with a cost "+bestCost.toFixed(3)+" found! (Improvement: "+improvement.toFixed(3)+"%)")
        storeRestoreActionValueDataMeans(1,bestPolicy);
    }

    
    // numOfRLIterations = 0;
}

function updateActionValueData(mode){
    // update the actionValueData based on the reward sequence recorded at each agent for the episode

    var discountFactor = RLDiscountFactor; //1

    for(var a = 0; a<agents.length; a++){
        var returnVal = 0;
        for(var k = agents[a].rewardSequence.length-1; k>=0; k--){
            var i = agents[a].rewardSequence[k][0];
            var j = agents[a].rewardSequence[k][1];
            var R = agents[a].rewardSequence[k][2];
            returnVal = discountFactor*returnVal + R; // discounted return (i.e., future sum of (discounted) rewards)
            agents[a].actionValueData[i][j].push(returnVal); // i is the state, j is the action
            
            var N_new = agents[a].actionValueData[i][j].length - 2;
            var mean_Old = agents[a].actionValueData[i][j][1]; // episode mean
            var x_new = returnVal;
            var mean_New = mean_Old + (x_new-mean_Old)/N_new;
            agents[a].actionValueData[i][j][1] = mean_New;
        }

        if(mode==1){
            // update overal control policy (baseed on episodic means)
            var updateStepSize;
            if(RLStepSizeType==0){
                updateStepSize = RLStepSizeValue//1/(1+numOfRLIterations); //should be between 0 and 1 (slow vs fast, respectively)
            }else if(RLStepSizeType==1){
                updateStepSize = RLStepSizeValue/Math.sqrt(numOfRLIterations+1);
            }else{
                updateStepSize = RLStepSizeValue/(RLStepSizeValue+numOfRLIterations);
            }
            

            for(var i=0; i<targets.length; i++){
                for(var j=0; j<targets.length; j++){
                    if(agents[a].actionValueData[i][j].length>2){
                        var C_ijOld = agents[a].actionValueData[i][j][0];
                        var C_ijNew = agents[a].actionValueData[i][j][1];
                        C_ijNew = C_ijOld + updateStepSize*(C_ijNew - C_ijOld);
                        agents[a].actionValueData[i][j][0] = C_ijNew;
                    }
                }
            }
        }


    }

}

function storeRestoreActionValueDataMeans(mode,means){
    var matVal = [];
    for(var a=0; a<agents.length; a++){
        if(mode==0){matVal.push([])}
        
        for(var i=0; i<targets.length; i++){
            if(mode==0){matVal[a].push([])}
            
            for(var j=0; j<targets.length; j++){
                if(mode==0){//store: retuen the current means
                    matVal[a][i].push([]);
                    matVal[a][i][j] = agents[a].actionValueData[i][j][0];
                }else{// restore using 
                    agents[a].actionValueData[i][j] = [means[a][i][j],0]; 
                }

            }
        }
    }
    if(mode==0){return matVal;}

}


// function updateActionValueData(){
//     // update the actionValueData based on the reward sequence recorded at each agent for the episode

//     var discountFactor = 1; //1

//     for(var a = 0; a<agents.length; a++){
//         var returnVal = 0;
//         for(var k = agents[a].rewardSequence.length-1; k>=0; k--){
//             var i = agents[a].rewardSequence[k][0];
//             var j = agents[a].rewardSequence[k][1];
//             var R = agents[a].rewardSequence[k][2];
//             returnVal = discountFactor*returnVal + R; // discounted return (i.e., future sum of (discounted) rewards)
//             agents[a].actionValueData[i][j].push(returnVal); // i is the state, j is the action
            
//             var N_new = agents[a].actionValueData[i][j].length - 2;
//             var mean_Old = agents[a].actionValueData[i][j][1]; // episode mean
//             var x_new = returnVal;
//             var mean_New = mean_Old + (x_new-mean_Old)/N_new;
//             agents[a].actionValueData[i][j][1] = mean_New;
//         }

//         // update overal control policy (baseed on episodic means)
//         var updateStepSize = 1; //1/sq(1+numOfRLIterations); //should be between 0 and 1 (slow vs fast, respectively)
//         for(var i=0; i<targets.length; i++){
//             for(var j=0; j<targets.length; j++){
//                 if(agents[a].actionValueData[i][j].length>2){
//                     var C_ijOld = agents[a].actionValueData[i][j][0];
//                     var C_ijNew = agents[a].actionValueData[i][j][1];
//                     C_ijNew = C_ijOld + updateStepSize*(C_ijNew - C_ijOld);
//                     agents[a].actionValueData[i][j][0] = C_ijNew;
//                 }
//             }
//         }


//     }

// }
// function tuneRLBasedEDRHC(numOfIterations){
//     // var numOfIterations = 100;
//     var costArray = [];
//     var paraArray = [];
//     var bestCost = Infinity;
//     var bestPolicy;
//     var meanVals = [];
//     for(k = 0; k<numOfIterations; k++){
//         simulateHybridSystemFast();    
//         print("Cost at k="+k+", is :"+terminalMeanSystemUncertainty.toFixed(3));
        
//         costArray.push(terminalMeanSystemUncertainty);
//         paraArray.push(k);
        
//         if(terminalMeanSystemUncertainty<bestCost){
//             bestCost = terminalMeanSystemUncertainty;
//             bestPolicy = [...meanVals];
//             //print(bestPolicy[0][0][1])
//         }

//         updateActionValueData();
//         meanVals = storeRestoreActionValueDataMeans(0,0);  //stremode,dummy  
//         storeRestoreActionValueDataMeans(1,meanVals);
        
//         resetSimulation();
//         // print(agents[0].actionValueData[0][1])
//         numOfRLIterations++;    
//     }
    
//     plotCostVsParameterData(7,paraArray,costArray)
    

//     print("Best Cost = "+bestCost.toFixed(3));
//     storeRestoreActionValueDataMeans(1,bestPolicy);
//     // numOfRLIterations = 0;
// }


// function storeRestoreActionValueDataMeans(mode,means){
//     var matVal = [];
//     for(var a=0; a<agents.length; a++){
//         if(mode==0){matVal.push([])}
        
//         for(var i=0; i<targets.length; i++){
//             if(mode==0){matVal[a].push([])}
            
//             for(var j=0; j<targets.length; j++){
//                 if(mode==0){//store: retuen the current means
//                     matVal[a][i].push([]);
//                     matVal[a][i][j] = agents[a].actionValueData[i][j][0];
//                 }else{// restore using 
//                     agents[a].actionValueData[i][j] = [means[a][i][j],0]; 
//                 }

//             }
//         }
//     }
//     if(mode==0){return matVal;}

// }
