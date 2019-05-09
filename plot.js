function plotData(){

   	//constructSpaceForPlots();
	

   	if(costPlotMode){
		var trace1 = {x:updateStepCountArray, y: costArrayForPlot, type: 'scatter', line: {shape: 'hv'}};
	   	
	   	var plotLayout1 = 
			{
				title: 'Evolution of Objective Function Value', 
				////autosize: true,
			    //width: 1000,
			    //height: 400,
			    ////automargin: true,
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
				
	   		}

		var myPlot1 = document.getElementById('myPlot1');
				////Plotly.newPlot(myPlot, data, plotLayout,{displayModeBar: false});
		Plotly.newPlot(myPlot1, [trace1], plotLayout1);
	}



}


