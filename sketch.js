/*
// Dedicated to all the victims of Easter Bombings - Sri Lanka 
// 21 April 2019
*/

var canvas;
var width;
var height;


var saveSketch = false;

function setup() {
  	

    Math.seedrandom('9'); // initial seed for random number generator


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
                ////targets[i].updateCT2(); // IPA debug purposes
                targets[i].updateCT(); // update uncertainty levels
                cost = cost + targets[i].meanUncertainty;
            }

            
            for(var i = 0; i < agents.length; i++){
                ////agents[i].updateCT2(); // IPA debug purposes
                agents[i].updateCT(); // update positions of the agents  
            }

            simulationTime = simulationTime + deltaT;
            discreteTimeSteps = discreteTimeSteps + 1;

            var t = simulationTime;

            // Randomization 4: random uncertainty perturbation
            if(RHCNoiseEnabled&&RHCNoiseR_iMagnitude>0){
                for(var i = 0; i < targets.length; i++){
                    targets[i].updateRHCNoiseR_i(); // update uncertainty levels
                }
            }
            // Randomization 4: random uncertainty perturbation end


        }

        document.getElementById("simulationTime").innerHTML = t.toFixed(2).toString();
        document.getElementById("simulationCost").innerHTML = cost.toFixed(3).toString();

        //consolePrint('Cost: '+cost.toFixed(3).toString())
        ////print("Time: "+Math.round(simulationTime));
        ////print("Cost: "+cost);

    }else if(simulationMode==6){// RHC methods

        for(var k=0; k<simulationFrameRate; k++){
            
            var cost = 0;
            for(var i = 0; i < targets.length; i++){
                targets[i].updateCT(); // update uncertainty levels
                cost = cost + targets[i].meanUncertainty;
            }

            
            for(var i = 0; i < agents.length; i++){
                // the following line is the only difference compared to "SimulationMoode1" given above
                agents[i].updateRHCCT(); // update positions of the agents  
            }

            simulationTime = simulationTime + deltaT;
            discreteTimeSteps = discreteTimeSteps + 1;

            var t = simulationTime;

            // Randomization 4: random uncertainty perturbation
            if(RHCNoiseEnabled&&RHCNoiseR_iMagnitude>0){
                for(var i = 0; i < targets.length; i++){
                    targets[i].updateRHCNoiseR_i(); // update uncertainty levels
                }
            }
            // Randomization 4: random uncertainty perturbation end


        }

        document.getElementById("simulationTime").innerHTML = t.toFixed(2).toString();
        document.getElementById("simulationCost").innerHTML = cost.toFixed(3).toString();

        //consolePrint('Cost: '+cost.toFixed(3).toString());
        ////print("Time: "+Math.round(simulationTime));
        ////print("Cost: "+cost);
    }else if(simulationMode==7){// Event Driven RHC methods
        
        for(var k=0; k<simulationFrameRate; k++){
            
            var cost = 0;
            for(var i = 0; i < targets.length; i++){
                targets[i].updateCT(); // update uncertainty levels
                cost = cost + targets[i].meanUncertainty;
            }

            var energyCost = 0;
            for(var i = 0; i < agents.length; i++){
                // the following line is the only difference compared to "SimulationMoode1" given above
                if(RHCMethod<8){
                    agents[i].updateEDRHCCT(); // update positions of the agents  
                }else if(RHCMethod<12){
                    agents[i].updateEDORHCCT(); // update positions of the agents  
                    energyCost = energyCost + agents[i].energySpent;
                }else if(RHCMethod<13){// update using clasifiers
                    agents[i].updateClassifierCT();
                }else if(RHCMethod<15){// update randomly
                    agents[i].updateRandomCT();
                }else if(RHCMethod==15){// RL method
                    agents[i].updateEDRHCCT();
                }
            }

            simulationTime = simulationTime + deltaT;
            discreteTimeSteps = discreteTimeSteps + 1;

            var t = simulationTime;

            // Randomization 4: random uncertainty perturbation
            if(RHCNoiseEnabled&&RHCNoiseR_iMagnitude>0){
                for(var i = 0; i < targets.length; i++){
                    targets[i].updateRHCNoiseR_i(); // update uncertainty levels
                }
            }
            // Randomization 4: random uncertainty perturbation end

        }

        if(RHCMethod<8 || RHCMethod>=12){
            document.getElementById("simulationTime").innerHTML = t.toFixed(2).toString();
            document.getElementById("simulationCost").innerHTML = cost.toFixed(3).toString();
        }else{
            document.getElementById("simulationTime2").innerHTML = t.toFixed(2).toString();
            document.getElementById("simulationCost2").innerHTML = cost.toFixed(1).toString();
            
            document.getElementById("agentEnergyCost").innerHTML = energyCost.toExponential(2).toString();
            var totalCost = cost + RHCalpha2*energyCost;
            document.getElementById("totalCost").innerHTML = totalCost.toFixed(1).toString();
        }
        //consolePrint('Cost: '+cost.toFixed(3).toString());
        ////print("Time: "+Math.round(simulationTime));
        ////print("Cost: "+cost);
    }else if(simulationMode==4){// IPA iterations

        if(numberOfUpdateStepsCount<numberOfUpdateSteps){        
            solveForIPAEstimators();
            simulationMode = 4;
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
            
        }

    }

    if(RGCComputingMode == 1.5){
        sleepFor(50);
        RGCComputingMode = 1;
    }else if(RGCComputingMode==1){
        // extend each cycle!
        var RGCComputingModeCount = 0; 
        
        for(var j = 0; j < agents.length; j++){
            if(!cycleGenerationMethod){
                cycles[j].iterationOfComputingGreedyCycles(50);  
            }else{
                cycles[j].iterationOfComputingGreedyCyclesAdvanced(50);  
            }
            if(cycles[j].RGCComputingMode==0){RGCComputingModeCount++;}  
        }

        if(RGCComputingModeCount==agents.length){// all cycles complete
            consolePrint('Theoretical Mean System Uncertainty = '+evaluateMeanSystemUncertainty().toFixed(3)+".");
            RGCComputingMode = 2;
        }

    }else if(RGCComputingMode==3){
        executeABargainingStep();
    }




    for(var i = 0; i < paths.length; i++){
    	paths[i].show();
    }

    for(var i = 0; i < cycles.length; i++){
        if(RGCComputingMode>0){
            cycles[i].show();    
        }
        
    }

    for(var i = 0; i < targets.length; i++){
    	targets[i].show();
    }

    for(var i = 0; i < agents.length; i++){
    	agents[i].show();
    }


     

    /// for J3 saving canvas

    if(RGCComputingMode==3){
               //save(); 
        if(cycleGenerationMethod){

            //save();
        }
    }

    if(saveSketch){
        ////save();
        saveSketch   = false;
    }


    // RHC random tests
    if(repeatedRandomTestMode>0){
        save("Test"+repeatedRandomTestMode); 
        var ans = runARandomTest(repeatedRandomTestMode);
        repeatedRandomTestData = repeatedRandomTestData + ans;
        repeatedRandomTestMode--;


        if(repeatedRandomTestMode==0){
            doSave(repeatedRandomTestData, "text", "Test"+repeatedRandomTestMode+"Data.txt");
        }else{
            // refreshRandomProblemConfiguration(5,1,300);
            refreshRandomProblemConfiguration(7,2,150);
            // refreshRandomProblemConfiguration(15,3,300);
            var count = 0;
            while(!isFullyConnected()){
                refreshRandomProblemConfiguration(7,2,150);
                if(count>10000){
                    print("Error")
                    break

                }
            }
        }


    }

    
    // if(justInitialized<10){
    //     justInitialized++;
    // }else if(justInitialized==10){
    //     readInitialInterface(); 
    //     justInitialized=100;
    // }
    

}


