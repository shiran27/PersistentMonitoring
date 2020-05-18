function plotData(){

   	//constructSpaceForPlots();
	

   	if(costPlotMode){
		var trace1 = {x:updateStepCountArray, y: costArrayForPlot, 
						type: 'scatter', line: {shape: 'hv'}, name:'Normal'};
	   	var trace2 = {x:updateStepCountArray, y: boostedCostArrayToPlot, 
	   					 type: 'scatter', yaxis: 'y2', line: {shape: 'hv'}, name:'Boosting'};
      
	   	var plotLayout1 = 
			{
				title: 'Evolution of Objective Function Value', 
				////autosize: true,
			    //width: 100,
			    //height: 400,
			    automargin: true,
			    // margin: {
			    // 	l:10,
			    // 	r:10,
			    // 	t:10,
			    // 	b:10,
			    // 	pad:10,
			    // },
			    xaxis: {
					title: 'Number of Iterations', 
					showline: true,
					showgrid: true, 
					zeroline: true,
					////automargin: true,
				}, 
				yaxis: {
					title: 'Objective - J', 
				    showline: true,
				    showgrid: true,
				    zeroline: true,
				    ////automargin: true,
				},
				yaxis2: {
					title: 'Boosted Objective', 
				    showline: true,
				    showgrid: true,
				    zeroline: true,	
				    titlefont: {color: '#ff7f0e'},
    				tickfont: {color: '#ff7f0e'},
    				overlaying: 'y',
    				side: 'right'
				},
				
	   		};

		var myPlot1 = document.getElementById('myPlot1');
				////Plotly.newPlot(myPlot, data, plotLayout,{displayModeBar: false});
		Plotly.newPlot(myPlot1, [trace1,trace2], plotLayout1);
	}



}

function plotCostVsParameterData(paraType,dataX,dataY){

   	//constructSpaceForPlots();
	var xLabel;
	if(paraType==0){
		xLabel = 'Simulation Period: T'
	}else if(paraType==1){
		xLabel = 'Planning Horizon: H'
	}else if(paraType==2){
		xLabel = 'RHC Parameter: Alpha'
	}else if(paraType==3){
		xLabel = 'RHC Parameter: Beta'
	}else if(paraType==4){
		xLabel = 'Realization No.'
	}

	var trace1 = {x:dataX, y:dataY,type: 'scatter', name:'Normal'};
	   	
	var plotLayout1 = 
	{
		title: 'Objective Function Value Vs Paramter Value', 
		////autosize: true,
	    //width: 100,
	    //height: 400,
	    automargin: true,
	    // margin: {
	    // 	l:10,
	    // 	r:10,
	    // 	t:10,
	    // 	b:10,
	    // 	pad:10,
	    // },
	    xaxis: {
			title: xLabel, 
			showline: true,
			showgrid: true, 
			zeroline: true,
			////automargin: true,
		}, 
		yaxis: {
			title: 'Objective - J', 
		    showline: true,
		    showgrid: true,
		    zeroline: true,
		    ////automargin: true,
		},
	}
	
	var myPlot1 = document.getElementById('myPlot1');
				
	Plotly.newPlot(myPlot1, [trace1], plotLayout1);
	
}







function plotAdditionalData(meanUncertainty){
	// save data 
	window.localStorage.setItem('targetStateData', JSON.stringify(targetStateData));
	window.localStorage.setItem('agentStateData', JSON.stringify(agentStateData));
	window.localStorage.setItem('eventTimeData', JSON.stringify(eventTimeData));
	window.localStorage.setItem('meanUncertainty', JSON.stringify(meanUncertainty));
	//window.focus();

}

function openPlotsPage(){
	var newWindow = window.open('plot.html');
}

function customizeTargetSelection(val){
	var dataString1 = localStorage.getItem('targetStateData');
	var targetStateDataObtained = JSON.parse(dataString1);
	var M = targetStateDataObtained.length;

	var stringForTargetCheckBoxes = "";
	for(var i = 0; i<M; i++){
		stringForTargetCheckBoxes = stringForTargetCheckBoxes + "&nbsp;&nbsp;&nbsp;<label for='targetSelect"+i+"'>T_"+(i+1)+": </label>";
		if(val==1){stringForTargetCheckBoxes = stringForTargetCheckBoxes + "<input type='checkbox' id='targetSelect"+i+"' onchange='plotTargetStateData();' checked>";}
		else{stringForTargetCheckBoxes = stringForTargetCheckBoxes + "<input type='checkbox' id='targetSelect"+i+"' onchange='plotTargetStateData();'>";}
	}
	document.getElementById("targetSelectCheckBoxes").innerHTML = stringForTargetCheckBoxes;

	plotTargetStateData();
}

function customizeAgentSelection(val){
	var dataString1 = localStorage.getItem('agentStateData');
	var agentStateDataObtained = JSON.parse(dataString1);
	var N = agentStateDataObtained.length;

	var stringForAgentCheckBoxes = "";
	for(var i = 0; i<N; i++){
		stringForAgentCheckBoxes = stringForAgentCheckBoxes + "&nbsp;&nbsp;&nbsp;<label for='agentSelect"+i+"'>A_"+(i+1)+": </label>";
		if(val==1){stringForAgentCheckBoxes = stringForAgentCheckBoxes + "<input type='checkbox' id='agentSelect"+i+"' onchange='plotAgentStateData();' checked>";}
		else{stringForAgentCheckBoxes = stringForAgentCheckBoxes + "<input type='checkbox' id='agentSelect"+i+"' onchange='plotAgentStateData();'>";}
	}
	document.getElementById("agentSelectCheckBoxes").innerHTML = stringForAgentCheckBoxes;

	plotAgentStateData();
}



function plotTargetStateData(){

	if(document.getElementById("targetSelectCheckBoxes").innerHTML == ""){
		customizeTargetSelection();
	}


	var dataString1 = localStorage.getItem('targetStateData');
	var targetStateDataObtained = JSON.parse(dataString1);

	var dataString2 = localStorage.getItem('eventTimeData');
	var eventTimeDataObtained = JSON.parse(dataString2);

	var dataString3 = localStorage.getItem('meanUncertainty');
	var meanUncertaintyObtained = JSON.parse(dataString3);
	document.getElementById("meanUncertaintyDisplay").innerHTML = meanUncertaintyObtained.toFixed(3);


	var M = targetStateDataObtained.length;

	var traceArray = [];
	for(var i = 0; i<M; i++){
		if(document.getElementById("targetSelect"+i).checked){
			var stringName = 'Target ';
			stringName = stringName.concat(i+1);

			var trace = {x:eventTimeDataObtained, y:targetStateDataObtained[i], type: 'scatter', name:stringName}
			traceArray.push(trace);
		}
	}
   	// var trace1 = {x:updateStepCountArray, y: costArrayForPlot, 
				// 	type: 'scatter', line: {shape: 'hv'}, name:'Normal'};
   	
   	var plotLayout1 = 
		{
			title: 'Evolution of Target State', 
			////autosize: true,
		    //width: 100,
		    //height: 400,
		    automargin: true,
		    // margin: {
		    // 	l:10,
		    // 	r:10,
		    // 	t:10,
		    // 	b:10,
		    // 	pad:10,
		    // },
		    xaxis: {
				title: 'Time', 
				showline: true,
				showgrid: true, 
				zeroline: true,
				////automargin: true,
			}, 
			yaxis: {
				title: 'Target Uncertainty', 
			    showline: true,
			    showgrid: true,
			    zeroline: true,
			    ////automargin: true,
			},
			
   		};

	var myPlot1 = document.getElementById('targetStateDataPlotHolder');
			////Plotly.newPlot(myPlot, data, plotLayout,{displayModeBar: false});
	Plotly.newPlot(myPlot1, traceArray, plotLayout1);
	

}

function plotAgentStateData(){

	if(document.getElementById("agentSelectCheckBoxes").innerHTML == ""){
		customizeAgentSelection();
	}

	var dataString1 = localStorage.getItem('agentStateData');
	var agentStateDataObtained = JSON.parse(dataString1);

	var dataString2 = localStorage.getItem('eventTimeData');
	var eventTimeDataObtained = JSON.parse(dataString2);
	
	var N = agentStateDataObtained.length;

	var traceArray = [];
	for(var i = 0; i<N; i++){
		var stringName = 'Agent ';
		stringName = stringName.concat(i+1);

		if(document.getElementById("agentSelect"+i).checked){
			var trace = {x:eventTimeDataObtained, y:agentStateDataObtained[i], type: 'scatter', name:stringName}
			traceArray.push(trace);
		}
	}
   	// var trace1 = {x:updateStepCountArray, y: costArrayForPlot, 
				// 	type: 'scatter', line: {shape: 'hv'}, name:'Normal'};
   	
   	var plotLayout1 = 
		{
			title: 'Evolution of Agent State', 
			////autosize: true,
		    //width: 100,
		    //height: 400,
		    automargin: true,
		    // margin: {
		    // 	l:10,
		    // 	r:10,
		    // 	t:10,
		    // 	b:10,
		    // 	pad:10,
		    // },
		    xaxis: {
				title: 'Time', 
				showline: true,
				showgrid: true, 
				zeroline: true,
				////automargin: true,
			}, 
			yaxis: {
				title: 'Residing Target Index', 
			    showline: true,
			    showgrid: true,
			    zeroline: true,
			    ////automargin: true,
			},
			
   		};

	var myPlot1 = document.getElementById('agentStateDataPlotHolder');
			////Plotly.newPlot(myPlot, data, plotLayout,{displayModeBar: false});
	Plotly.newPlot(myPlot1, traceArray, plotLayout1);
	

}



