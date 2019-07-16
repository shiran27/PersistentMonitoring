function plotData(){

   	//constructSpaceForPlots();
	

   	if(costPlotMode){
		var trace1 = {x:updateStepCountArray, y: costArrayForPlot, 
						type: 'scatter', line: {shape: 'hv'}, name:'Normal'};
	   	var trace2 = {x:updateStepCountArray, y: boostedCostArrayToPlot,
	   					 type: 'scatter', line: {shape: 'hv'}, name:'Boosting'};
      
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
				
	   		};

		var myPlot1 = document.getElementById('myPlot1');
				////Plotly.newPlot(myPlot, data, plotLayout,{displayModeBar: false});
		Plotly.newPlot(myPlot1, [trace1,trace2], plotLayout1);
	}



}


