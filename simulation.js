
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
        for(var j = 0; j<targets.length; j++){
            if(isNeighbors(i,j)){
                targets[i].neighbors.push(j);// targets id's
            }
        }

        
        var indexString = (i+1).toString();
        var HTMLTag = "<h5>Target "+ indexString +":";
        HTMLTag += "<input type='range' min='0' max='15' step='0.2' value='"+targets[i].uncertaintyRate.toFixed(1)+"' class='slider' id='uncertaintyRate"+ indexString +"' onchange='uncertaintyRateChangedRT("+indexString+",this.value)'> <span id='uncertaintyRateDisplay"+ indexString +"'></span></h5>";
        document.getElementById('customRTUncertaintyRates').innerHTML += HTMLTag;         


        var HTMLTag2 = "<td contenteditable='true' id='customRTUncertainty"+indexString+"'>"+targets[i].uncertainty+"</td>";
        document.getElementById('customRTUncertainty').innerHTML += HTMLTag2; 

    }



    document.getElementById('customRTSensingRates').innerHTML  = "";
    document.getElementById('customRTThreshold').innerHTML = "";
    document.getElementById('thresholdSensitivities').innerHTML = "";
    var HTMLTag2 = "<div class='col'></div>";    
    var HTMLTag1 = "";
    for(var a = 0; a<agents.length; a++){

        var indexString = (a+1).toString();
        var HTMLTag = "<h5>Agent "+ indexString +":";
        HTMLTag += "<input type='range' min='0' max='15' step='0.1' value='"+agents[a].sensingRate.toFixed(1)+"' class='slider' id='sensingRate"+ indexString +"' onchange='sensingRateChangedRT("+indexString+",this.value)'> <span id='sensingRateDisplay"+ indexString +"'></span></h5>";
        HTMLTag1 += HTMLTag;

        //thresholds per each agent
        HTMLTag2 += "<table class='col table table-bordered matrix' id='customRTThreshold"+(a+1)+"'>";
        agents[a].threshold = [];
        for(var i = 0; i<targets.length; i++){//rows
            agents[a].threshold[i] = [];
            
            HTMLTag2 += "<tr>";
            for(var j = 0; j<targets.length; j++){//columns
                
                if( i!=j && !isNeighbors(i,j)){
                    var threshold = 10000;    
                }else{
                    var threshold = Math.round(100*10*Math.random())/100;
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

    }

}

function readInitialInterface(){
    deltaT = Number(document.getElementById("deltaT").value);
    periodT = Number(document.getElementById("periodT").value);
    stepSize = Number(document.getElementById("stepSize").value)*Math.pow(10,Number(document.getElementById("stepSizeMultiplier").value));
    numberOfUpdateSteps = Number(document.getElementById("numberOfUpdateSteps").value);
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

    consolePrint("Obtained threshold sensitivities were loaded to the display.")
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


function sensingRateChangedRT(index,val){
    consolePrint("Agent "+index+"'s sensing rate changed to "+val+".")
    agents[index-1].sensingRate = val;
}


function uncertaintyRateChangedRT(index,val){
    consolePrint("Target "+index+"'s uncertainty rate changed to "+val+".")
    targets[index-1].uncertaintyRate = val;
}


function sensingRateChanged(val){
    if(agentSelectDropdownUsed){
        agentSelectDropdownUsed = false;
    }else{
        var val1 = Number(document.getElementById("agentSelectDropdown").value); 
        if(isNaN(val1)){
            for(var i = 0; i<agents.length; i++){// all agents reset to default
                agents[i].sensingRate = val;
            }
        }else{
            agents[val1-1].sensingRate = val;
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
                targets[i].uncertaintyRate = val;
            }
        }else{
            targets[val1-1].uncertaintyRate = val;
        }
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

    consolePrint("Step size of the gradient descent scheme changed to: "+stepSize+".");
    
    document.getElementById("stepSizeDisplay").innerHTML = stepSize.toString();
    document.getElementById("stepSizeMultiplierDisplay").innerHTML = "10<sup>"+stepSizeMultiplier+"</sup>";
}





function simulateHybridSystem(){ // run the hybrid system indefinitely in real-time while displaying

    consolePrint("Initiated simulating the hybrid system indefinitely under the given threshold policy.");

    simulationMode = 1;
    simulationTime = 0;
    discreteTimeSteps = 0;


}



function simulateHybridSystemFast(){ // run the hybrid system for time T period (without displaying agent movements) and get the IPA estimtors

    consolePrint("Initiated running the hybrid system for a time period T at once.");

    simulationMode = 2;
    simulationTime = 0;
    discreteTimeSteps = 0;


    for(var i = 0; i<targets.length; i++){// rest targets
        targets[i].uncertainty = targets[i].initialUncertainty;
        targets[i].meanUncertainty = targets[i].initialUncertainty;
    }

    while(simulationTime < periodT){
    
        for(var i = 0; i < targets.length; i++){
            targets[i].updateFastCT(); // update uncertainty levels
        }


        for(var i = 0; i < agents.length; i++){
            agents[i].updateFastCT(); // update positions of the agents
        }

        simulationTime = simulationTime + deltaT;
        discreteTimeSteps = discreteTimeSteps + 1;
        ////print(simulationTime);
    }

    var meanUncertainty = 0;
    for(var i = 0; i<targets.length; i++){
        targets[i].meanUncertainty = (targets[i].meanUncertainty - 0.5*(targets[i].uncertainty + targets[i].initialUncertainty))/discreteTimeSteps;
        meanUncertainty = meanUncertainty + targets[i].meanUncertainty;
    }


    // resetAgentsAndTargets();
    for(var i = 0; i<agents.length; i++){// rest agent positions
        if(agents[i].residingTarget.length==1){
            agents[i].position = targets[agents[i].residingTarget[0]].position;    
        }else if(agents[i].residingTarget.length==2){
            agents[i].position = targets[agents[i].residingTarget[1]].position;
            agents[i].residingTarget = [agents[i].residingTarget[1]];
        }else{

        }
        // agents[i].orientation = 0;
        // agents[i].graphicBaseShapeRotated = agents[i].graphicBaseShape; 

    }

    
    document.getElementById("simulationTime").innerHTML = simulationTime.toFixed(2).toString();
    document.getElementById("simulationCost").innerHTML = meanUncertainty.toFixed(3).toString();


    print("Cost : "+meanUncertainty);
    simulationMode = 0;

}


function solveForIPAEstimators(){ // run the hybrid system for time T period (without displaying agent movements) and get the IPA estimtors

    consolePrint("Initiated running the hybrid system for time perion T and getting IPA estimators.");

    simulationMode = 3;
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
        simulationTime = Number(simulationTime.toFixed(2));
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


    consolePrint("Cost : "+meanUncertainty.toFixed(3));
    simulationMode = 0;
}


function sensitivityUpdateAtEvent(){

    var eventTimePeriod = simulationTime - eventTime;
    eventTime = simulationTime;    
    eventCount = eventCount + 1; 
 
    if(eventCount==1){
        firstEventTime = eventTime;
        print("First event of the simulation occured at: "+eventTime);
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
    }

    for(var i = 0; i<agents.length; i++){// rest agent positions
        agents[i].residingTarget = [agents[i].initialResidingTarget];
        agents[i].position = targets[agents[i].residingTarget[0]].position;
        
    }

    consolePrint("Simulation stopped and reseted to the initial state.");
    

}



function optimizeThresholds(){// use solveForIPAEstimators() iteratively to updte the thresholds 

    consolePrint("Finding optimal threshold policy using IPA estimators started.");
    simulationMode = 4;
    costArrayForPlot = [];
    updateStepCountArray = [];
    numberOfUpdateStepsCount = 0;

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

function updateThresholdPolicy(){
    // stepSize is a global variable

    for(var z = 0; z<agents.length; z++){
        for(var p = 0; p < targets.length; p++){
            for(var q = 0; q < targets.length; q++){
                var val = agents[z].threshold[p][q] - stepSize*agents[z].sensitivityOfThreshold[p][q];
                if(val<=0){
                    val = 0;
                }else if(val>10000){
                    val = 10000;
                } 
                agents[z].threshold[p][q] = val;
            }
        }
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



function pauseSimulation(){
    if(simulationMode == 0){
        simulationMode = 1;
        document.getElementById("pauseButton").innerHTML = "<i class='fa fa-pause' aria-hidden='true'></i>";     
        consolePrint("Hybrid system simulation started.");
    }else{
        simulationMode = 0;
        document.getElementById("pauseButton").innerHTML = "<i class='fa fa-play' aria-hidden='true'></i>"; 
        consolePrint("Hybrid system simulation paused.");
    }
    
}

function stopSimulation(){
    
    simulationMode = 0;
    simulationTime = 0;
    discreteTimeSteps = 0;

    resetAgentsAndTargets();

    consolePrint("Simulation stopped and reseted to the initial state.");

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
    
    startModifyingProbConfig();
    for(var i = agents.length; i > 0; i--){
        removeAnAgent();
    }
    for(var i = targets.length; i > 0; i--){
        removeATarget();
        print(targets.length)
    }
    print(targets);
    print(paths);
    //paths = [];
    finishModifyingProbConfig();


    if(r==2){//2A arrangement
        /*addATargetAt(50,50);
        addATargetAt(50,200);
        addATargetAt(550,50);
        addATargetAt(300,400);
        addATargetAt(150,200);*/
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
        startModifyingProbConfig();
        addATargetAt(50,450);
        addATargetAt(50,300);
        addATargetAt(250,450);
        addAnAgentAtTarget(0);
        finishModifyingProbConfig();
    }
}