/*
// Dedicated to all the victims of Easter Bombings - Sri Lanka 
// 21 April 2019
*/

var canvas;
var width;
var height;

function setup() {
  	

    Math.seedrandom('7'); // initial seed for random number generator


  	pixelDensity(1);
    const canvasHolder = select('#canvasHolder');
    width  = canvasHolder.width;
    height = canvasHolder.height;
    canvas = createCanvas(width, height);
    canvas.parent('canvasHolder');


    // target and agent select dropdowns
    var option1 = document.createElement("option");
    option1.text = "All";
    document.getElementById("targetSelectDropdown").add(option1);
    document.getElementById("targetSelectDropdown").selectedIndex = 0;

    var option1 = document.createElement("option");
    option1.text = "All";
    document.getElementById("agentSelectDropdown").add(option1);
    document.getElementById("agentSelectDropdown").selectedIndex = 0;


    // display
    document.getElementById("editAgentsWizard").style.display = "none";
    document.getElementById("editTargetsWizard").style.display = "none";

    document.getElementById("spaceForEditWizard0").style.display = "none";
    document.getElementById("spaceForEditWizard1").style.display = "none";

    document.getElementById("spaceForRealTimeUncertaintyRateEdit").style.display = "none";
    document.getElementById("spaceForRealTimeSensingRateEdit").style.display = "none";
    document.getElementById("spaceForRealTimeUncertaintyEdit").style.display = "none";
    document.getElementById("spaceForRealTimeThresholdEdit").style.display = "none";
    document.getElementById("spaceForThresholdSensitivities").style.display = "none";

    problemConfigurationChanged();

    consolePrint("Interface Loaded.");

    readInitialInterface();
    frameRate(1/deltaT);
    



}

function draw() {
	

	background(225);
    strokeWeight(4);
    noFill();
    stroke(0);
    rect(0,0,width,height);



    updateInterface();



    if(simulationMode==1){// Hybrid system simulation mode

        for(var k=0; k<simulationFrameRate; k++){
            
            var cost = 0;
            for(var i = 0; i < targets.length; i++){
                targets[i].updateCT(); // update uncertainty levels
                cost = cost + targets[i].meanUncertainty;
            }

            
            for(var i = 0; i < agents.length; i++){
                agents[i].updateCT(); // update positions of the agents
                
            }

            simulationTime = simulationTime + deltaT;
            discreteTimeSteps = discreteTimeSteps + 1;

            var t = simulationTime;

        }

        document.getElementById("simulationTime").innerHTML = t.toFixed(2).toString();
        document.getElementById("simulationCost").innerHTML = cost.toFixed(3).toString();

        ////print("Time: "+Math.round(simulationTime));
        ////print("Cost: "+cost);
    }else if(simulationMode==4){

        if(numberOfUpdateStepsCount<numberOfUpdateSteps){        
            solveForIPAEstimators();
            simulationMode = 4;
            updateThresholdPolicy(); 
            displayThresholds();
            numberOfUpdateStepsCount = numberOfUpdateStepsCount + 1;
            var cost = Number(document.getElementById("simulationCost").innerHTML);
            costArrayForPlot.push(cost);
            updateStepCountArray.push(numberOfUpdateStepsCount);
            plotData();
            consolePrint("Iteration "+ numberOfUpdateStepsCount+ " completed. Cost: "+cost+".");
        }

    }






    for(var i = 0; i < paths.length; i++){
    	paths[i].show();
    }


    for(var i = 0; i < targets.length; i++){
    	targets[i].show();
    }

    for(var i = 0; i < agents.length; i++){
    	agents[i].show();
    }



    
    // if(justInitialized<10){
    //     justInitialized++;
    // }else if(justInitialized==10){
    //     readInitialInterface(); 
    //     justInitialized=100;
    // }
    

}


