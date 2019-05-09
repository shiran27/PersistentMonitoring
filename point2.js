function Point2(x,y) {

	this.x = x;
	this.y = y; 
	
	this.lengthP2 = function(){
		var val = Math.sqrt(this.x*this.x + this.y*this.y);
		return val;
	}
	

	this.setP2 = function(x,y){
		this.x = x;
		this.y = y;
	}

	this.copyP2 = function(a){
		this.x = a.x;
		this.y = a.y;
	}

	this.shiftP2 = function(delta_x, delta_y){
		this.x += delta_x;
		this.y += delta_y;
	}

	this.cloneP2 = function(){
		return new Point2(this.x,this.y);
	}
	
	this.colorP2 = function(){
		return get(this.x,this.y);
	}
	
	this.pixelColorP2 = function(){
		var offset = (Math.round(this.y)*width + Math.round(this.x)) * 4; 
		//print(pixelMap.length)
		return [ pixelMap[offset], pixelMap[offset + 1], pixelMap[offset + 2], pixelMap[offset + 3] ]; 
	}	

}

//print points

function printPointArrayP2(pointArray,colorIn,r){
	
	//fill(128,0,0);
	fill(colorIn);
	stroke(colorIn);
	//stroke(128,0,0);
	for(var i=0 ; i<pointArray.length; i++){
	
		ellipse(pointArray[i].x, pointArray[i].y, r, r);
		
	}
	
	
}


//function to find whether a given point is inside or outside the canvas

function isOutOfCanvas(interestedPoint){
	
	if(width==height){	
		return (Math.max(interestedPoint.x,interestedPoint.y) > width || Math.min(interestedPoint.x,interestedPoint.y) < 0);
	}
	else{
		return ( (interestedPoint.x > width || interestedPoint.y > height) || Math.min(interestedPoint.x,interestedPoint.y) < 0 );
	}
	
}


// function to find whether the point is way way out of the canvas (intersectiong two edges)
function isWayOutOfCanvas(nextPosition){
	var guard = 20;
	if(nextPosition.x < 0 && nextPosition.y < 0){
		return [true, new Point2(guard,guard)];
	}
	else if(nextPosition.x < 0 && nextPosition.y > height){
		return [true, new Point2(guard,height-guard)];
	}
	else if(nextPosition.x > width && nextPosition.y > height){
		return [true, new Point2(width-guard,height-guard)];
	}
	else if(nextPosition.x > width && nextPosition.y < 0){
		return [true, new Point2(width-guard,guard)];
	}
	else {
		return [false, nextPosition];
			
	}

}






// extending from point in the given direction till end of sensor range
function pointsBeyondP2(a,b,stepSize){//use senRange to get z
	
	var unitVector = normalizeP2(minusP2(b,a));
	var pointArray = [];
	var pointArrayTemp = [];
	
	var z = plusP2(a,productP2(unitVector,senRange));
	var numOfSteps = Math.ceil(distP2(b,z)/stepSize);
	
	for(var i=0; i<numOfSteps; i++){
		pointArray.push(plusP2(b,productP2(unitVector,stepSize*(i+1))));
	}
	pointArray[pointArray.length-1] = z;//last element should be z

	//fwd check
	for(var i=0; i<numOfSteps; i++ ){
		var z_temp = pointArray[i];
		if(i<10){
			pointArrayTemp.push(pointArray[i]);
			//print("added");
		}
		else if(isOutOfCanvas(z_temp)){
			break;
		}
		else if(isColorEqualP2C(z_temp,obstacleColor)){
			for (var j = pointArrayTemp.length-1; j>=0; j--){//foward search
				if(isColorEqualP2C(pointArrayTemp[j],obstacleColor)){
					pointArrayTemp.pop();
				}
				else{
					break
				}
			}
			break;
		}
		else{
			pointArrayTemp.push(pointArray[i]);
		}
				
			
	}

	pointArray = pointArrayTemp;

	return pointArray;
	
	//check z feasibility
}

//extending from point in the given direction
function pointsInteriorP2(a,b,stepSize){
	
	var unitVector = normalizeP2(minusP2(b,a));
	var pointArray = [];
	var pointArrayTemp = [];
	
	var numOfSteps = Math.ceil(distP2(a,b)/stepSize);
	
	for(var i=0; i<numOfSteps; i++){
		pointArray.push(plusP2(a,productP2(unitVector,stepSize*(i+1))));
	}
	
	pointArray[pointArray.length-1] = b;
	
	for (var j = 0; j<pointArray.length; j++){//foward search
		if(!isColorEqualP2(b,pointArray[j])){
			pointArrayTemp.push(pointArray[j]);
		}
		else{
			break;
		}
	}
	
	
	return pointArrayTemp;
}


function getInteriorTrajectoryPointsP2(a,b,stepSize){//negilecting obstacles
	var unitVector = normalizeP2(minusP2(b,a));
	var pointArray = [];
		
	var numOfSteps = Math.ceil(distP2(a,b)/stepSize);
	
	for(var i=0; i<numOfSteps; i++){
		pointArray.push(plusP2(a,productP2(unitVector,stepSize*(i+1))));
	}
	
	pointArray[pointArray.length-1] = b;
	
	return pointArray
}



//pixel color matching between two points in the space
function isColorEqualP2(a,b){
	var col1 = a.pixelColorP2();
	var col2 = b.pixelColorP2();
	var count = 0;
	for(var i=0; i<col1.length; i++){
		if (col1[i]==col2[i]){
			count++;
		}
	}
	if(count==4){
		return true;
	}
	else{
		return false;
	}
}

//pixel color matching between a point ("a") and a mentioned color("b")
function isColorEqualP2C(a,b){
	//var col1 = a.colorP2();
	var col1 = a.pixelColorP2();
	var col2 = b.levels;
	var count = 0;
	for(var i=0; i<col1.length; i++){
		if (col1[i]==col2[i]){
			count++;
		}
	}
	if(count==4){
		return true;
	}
	else{
		return false;
	}
}


function isColorEqualP2CRigour(a,b){
	
	var aArray = [];
	var gridSize = 3;

	for(var x = -1; x<2; x++){
		for(var y = -1; y<2; y++){
			//print(x,y);
			var interestedPoint = new Point2(a.x+x*gridSize,a.y+y*gridSize);
			aArray.push(interestedPoint);
		}

	}
	//print(aArray);
	var countTotal = 0;

	for(var j=0; j<aArray.length; j++){
		
		if(isColorEqualP2C(aArray[j],b)){
			countTotal++;
		}

	}

	//print(countTotal)
	if(countTotal>=1){
		return true;
	}else{
		return false;
	}
}




//returns inermediate or extending point from b as a percentage of ||a-b||
//returns length(point2-point1)
function intermediateP2(a,b,lambda){
	//there is a gray area near vertices - 
	//var resultPoint = new Point2(lambda*a.x+(1-lambda)*b.x,lambda*a.y+(1-lambda)*b.y);
	//if (Math.floor(resultPoint.x)==b.x && Math.floor(resultPoint.y)==b.y){
		//print("hell");
	//}
	return new Point2(lambda*a.x+(1-lambda)*b.x,lambda*a.y+(1-lambda)*b.y);
	
}

function distP2(a,b){
	return Math.sqrt(Math.pow((b.x-a.x),2)+Math.pow((b.y-a.y),2));
}

//returns (point1+point2)/2
function middlePointP2(a,b){
	return new Point2((a.x+b.x)/2,(a.y+b.y)/2);
}


//returns point1/norm(point1)
function normalizeP2(a) {
    var foo = 1 / a.lengthP2();
    return new Point2(a.x*foo,a.y*foo);
}

//returns point2+point2
function plusP2(a, b){
    return new Point2(a.x+b.x,a.y+b.y);
}


//returns point2-point2
function minusP2(a, b){
    return new Point2(a.x-b.x,a.y-b.y);
}


//returns point2*double
function productP2(a, b){
    return new Point2(a.x*b,a.y*b);
}

//returns point2/double
function divideP2(a, b){
    return new Point2(a.x/b,a.y/b);
}

//dot product
function dotP2(a, b){
    return a.x * b.x + a.y * b.y;
}

//Norm
function normP2(a){
    return sqrt(sq(a.x)+sq(a.y));
}

//cross product
function crossP2(a, b){
    return a.x*b.y-b.x*a.y;
}

//angle of line between two point
function atan2P2(a, b){
    return Math.atan2(b.y-a.y,b.x-a.x);
}

//returns point1 == point2
function isEqualP2(a, b){
    if (a.x==b.x && a.y==b.y){
            return true;
    }
    else{
            return false;
    }
}

//returns rotated vector - counter clockwise
function rotateP2(a, angle){
	return new Point2(Math.cos(angle)*a.x-Math.sin(angle)*a.y,Math.cos(angle)*a.y+Math.sin(angle)*a.x);
}


function saturateP2(a,aMaxSq){
	if(dotP2(a, a) >= aMaxSq){ //a'a<aMaxSq
		return productP2(a,sqrt(aMaxSq/dotP2(a,a)));
	}
	else{
		return a;
	}
}

function avoidEscapeP2(a){
	var b = new Point2(a.x,a.y);
	var gurad = 5;

	if(a.x > width){
		b.x = width-gurad;
	}else if(a.x < 0){
		b.x = gurad;
	}
	if(a.y > height){
		b.y = height-gurad;
	}else if(a.y<0){
		b.y = gurad;
	}
	return b;

}

function repeatP2(a,N){
	result = [];
	for(i=0;i<N;i++){
		append(result,a);
	}
	return result;
}

