function RationalObj(A,B,C,D,E,F,G,H,K,P,L,Q,M,N){

    // numerator coefs
    this.A = A;
    this.B = B;
    this.C = C;
    this.D = D;
    this.E = E;
    this.F = F;
    
    // denominator coefs
    this.G = G;
    this.H = H;
    this.K = K;

    // constraints
    this.P = P;
    this.L = L;
    this.Q = Q;
    this.M = M;
    this.N = N;
    


    this.evaluateObj = function(x,y){
        var numerator =  this.A*sq(x) + this.B*sq(y) + this.C*x*y + this.D*x + this.E*y + this.F;
        var denominator = this.G*x + this.H*y + this.K;
        return numerator/denominator;
    }


    this.solveComplete = function(){

        var bestSol = [Infinity,0,0];

        // Search Along X
        var x_0 = 0;
        var y_0 = 0;
        var m = 0;
        var r_0 = 0;
        var r_1 = Math.min(this.N,(this.M/this.Q));
        var sol = this.solveLinOptX(x_0,y_0,m,r_0,r_1); //cost, x,y  
        ////print("Grads X:"+[this.evalGradX(x_0,y_0,m,r_0),this.evalGrad2X(x_0,y_0,m)])  
        if(sol[0]<bestSol[0]){bestSol = [...sol]}


        // Search Along Y
        var x_0 = 0;
        var y_0 = 0;
        var n = 0;
        var r_0 = 0;
        var r_1 = Math.min(this.L,this.M)
        var sol = this.solveLinOptY(x_0,y_0,n,r_0,r_1); //cost, x,y
        if(sol[0]<bestSol[0]){bestSol = [...sol]}     


        // Search Aling L
        if(this.L<this.M){
            var x_0 = 0;
            var y_0 = this.L;
            var m = this.P;
            var r_0 = 0;
            var r_1 = (this.M-this.L)/(this.P+this.Q)
            var sol = this.solveLinOptX(x_0,y_0,m,r_0,r_1); //cost, x,y
            ////print("Grads L:"+[this.evalGradX(x_0,y_0,m,r_0),this.evalGrad2X(x_0,y_0,m)])  
            if(sol[0]<bestSol[0]){bestSol = [...sol]}
        }



        // Search Along M
        var x_0 = 0;
        var y_0 = this.M;
        var m = -this.Q;
        var r_0 = Math.max(0,(this.M-this.L)/(this.P+this.Q));
        var r_1 = Math.min(this.N, this.M/this.Q)
        var sol = this.solveLinOptX(x_0,y_0,m,r_0,r_1); //cost, x,y  
        ////print("Grads M:"+[this.evalGradX(x_0,y_0,m,r_0),this.evalGrad2X(x_0,y_0,m)])  
        //// print("v_i="+sol[1]+"J="+sol[0])
        if(sol[0]<bestSol[0]){bestSol = [...sol]}  


        // Search Along N 
        if(this.N<(this.M/this.Q)){
            var x_0 = this.N;
            var y_0 = 0;
            var n = 0;
            var r_0 = 0;
            var r_1 = Math.min(this.M-this.Q*this.N);
            var sol = this.solveLinOptY(x_0,y_0,n,r_0,r_1); //cost, x,y
            if(sol[0]<bestSol[0]){bestSol = [...sol]} 
        }
        

        // KKT coditions - omitted for now!
        // this.formTheQuartic();
        // this.solveQuartic(); // x values
        // this.refineQuarticSolutions();


        return bestSol // cost, x, y values,: x will be executed!


    }


    this.solveLinOptX = function(x_0,y_0,m,r_0,r_1){

        var grad1 = this.evalGradX(x_0,y_0,m,r_0) // x_0,y_0,m, at r=r_0
        var grad2 = this.evalGrad2X(x_0,y_0,m) // x_0,y_0,m
        
        var opt_r = r_0;

        // see Fig.4 in P5
        if(grad1 <= 0 && grad2 <= 0){
            // r = r_1 is optimum
            opt_r = r_1;

        }else if(grad1 >= 0 && grad2 >= 0){
            // r = r_0 is optimum
            opt_r = r_0;

        }else if(grad1 < 0 && grad2 > 0){
            // compute r_crit1 (r where grad=0)
            var rCrit1 = this.evalrCrit1X(x_0,y_0,m);
            if(rCrit1 >= r_1){
                opt_r = r_1;
            }else{
                opt_r = rCrit1;
            }

        }else if(grad1 > 0 && grad2 < 0){
            // compute r_crit2 (r where grad=0)
            var rCrit2 = this.evalrCrit2X(x_0,y_0,m,r_0);
            if(rCrit2 >= r_1){
                opt_r = r_0;
            }else{
                opt_r = r_1;
            }

        }else{
            print("Error 1");
        }

        
        var x_opt = x_0+opt_r;
        var y_opt = y_0+m*opt_r;
        var J_opt = this.evaluateObj(x_opt,y_opt);
        return [J_opt, x_opt, y_opt]
        

        //return [Infinity,0,0]
    
    }


    this.solveLinOptY = function(x_0,y_0,n,r_0,r_1){// y_0 = 0, r_0 = 0

        var grad1 = this.evalGradY(x_0,y_0,n,r_0) // x_0,y_0,n, at r=r_0
        var grad2 = this.evalGrad2Y(x_0,y_0,n) // x_0,y_0,n
        
        var opt_r = r_0;

        // see Fig.4 in P5
        if(grad1 <= 0 && grad2 <= 0){
            // r = r_1 is optimum
            opt_r = r_1;

        }else if(grad1 >= 0 && grad2 >= 0){
            // r = r_0 is optimum
            opt_r = r_0;

        }else if(grad1 < 0 && grad2 > 0){
            // compute r_crit1 (r where grad=0)
            var rCrit1 = this.evalrCrit1Y(x_0,y_0,n);
            if(rCrit1 >= r_1){
                opt_r = r_1;
            }else{
                opt_r = rCrit1;
            }

        }else if(grad1 > 0 && grad2 < 0){
            // compute r_crit2 (r where grad=0)
            var rCrit2 = this.evalrCrit2Y(x_0,y_0,n,r_0);
            if(rCrit2 >= r_1){
                opt_r = r_0;
            }else{
                opt_r = r_1;
            }

        }else{
            print("Error 2");
        }


        var x_opt = x_0+n*opt_r;
        var y_opt = y_0+opt_r;
        var J_opt = this.evaluateObj(x_opt,y_opt);
        return [J_opt, x_opt, y_opt]
        
        //return [Infinity,0,0]

    }



    this.evalGradX = function(x_0,y_0,m,r_0){ // at r=r_0
       
        var r = r_0; // notational convinience!

        var sumVal = (this.D*this.K - this.F*this.G) + (this.E*this.K - this.F*this.H)*m;

        if(y_0!=0){
            sumVal = sumVal + (this.B*this.H*m + this.C*this.H - this.B*this.G)*sq(y_0) + (2*this.B*this.K*m + this.D*this.H - this.E*this.G + this.C*this.K)*y_0;
        } 
        // last 6 terms complete

        if(x_0!=0){
            sumVal = sumVal + ((this.C*this.G - this.A*this.H)*m + this.A*this.G)*sq(x_0) + (2*this.B*this.G*m*y_0 + 2*this.A*this.H*y_0 + (this.E*this.G - this.D*this.H + this.C*this.K)*m + 2*this.A*this.K)*x_0; 
        }
        // last 12 terms complete


        if(r != 0){
            sumVal = sumVal + (this.B*this.H*m*sq(m) + (this.B*this.G + this.C*this.H)*sq(m) + (this.A*this.H + this.C*this.G)*m + this.A*this.G)*sq(r);
            sumVal = sumVal + 2*(this.G*x_0+ this.H*y_0 + this.K)*(this.B*sq(m) + this.C*m + this.A)*r;
        }
        // all complete!

        return sumVal;
    }

    this.evalGrad2X = function(x_0,y_0,m){

        // every coef is divided by two!
        // terms that does not include x_0,y_0 or m
        var coef1 = this.F*sq(this.G) - this.D*this.G*this.K + this.A*sq(this.K);
        var sumVal = coef1;
        if(m!=0){
            var coef3 = this.F*sq(this.H) - this.E*this.H*this.K + this.B*sq(this.K);
            var coef2 = this.C*sq(this.K) + 2*this.F*this.G*this.H - this.D*this.H*this.K - this.E*this.G*this.K;
            sumVal = sumVal + m*( m*(coef3) + coef2 );
        }
        // last 3 coefs done

        var coef6 = 0; // same as sigma_1
        if(y_0!=0){
            coef6 = this.B*sq(this.G) - this.C*this.G*this.H + this.A*sq(this.H);
            var coef4 = this.E*sq(this.G) - this.D*this.G*this.H + 2*this.A*this.H*this.K - this.C*this.G*this.K; 

            if(m!=0){
                var coef5 = this.E*this.G*this.H - 2*this.B*this.G*this.K - this.D*sq(this.H) + this.C*this.H*this.K;
                coef4 = coef4 + m*coef5;
            }
            sumVal = sumVal + coef6*sq(y_0) + coef4*y_0;
        }
        // last 6 are done

        if(x_0!=0 && m!=0){
            var coef10 = coef6;
            var coef9 = -2*coef6;
            var coef8 = this.D*sq(this.H) + 2*this.B*this.G*this.K - this.E*this.G*this.H - this.C*this.H*this.K;
            var coef7 = this.D*this.G*this.H - this.E*sq(this.G) - 2*this.A*H*this.K + this.C*this.G*this.K;

            var mx = m*x_0;
            sumVal = sumVal + mx*(coef10*mx + coef9*y_0 + coef8*m + coef7);
        }

        return sumVal;
       
    }



    this.evalrCrit1X = function(x_0,y_0,m){
        
        // terms that does not include both x_0 or m
        var Aval = this.A*this.G;
        var Bval = 2*this.A*this.K + 2*this.A*this.H*y_0;
        var Cval = (this.D*this.K - this.F*this.G) + (this.D*this.H - this.E*this.G + this.C*this.K)*y_0 + (this.C*this.H - this.B*this.G)*sq(y_0);
 

        if(m!=0){ // terms that does not involve x_0
            Aval = Aval + this.B*this.H*sq(m)*m + (this.B*this.G + this.C*this.H)*sq(m) + (this.A*this.H + this.C*this.G)*m;
            Bval = Bval + 2*m*(this.H*y_0 + this.K)*(this.B*m + this.C); // this simplifies to give four terms 
            Cval = Cval + m*(this.B*this.H*sq(y_0) + 2*this.B*this.K*y_0 + this.E*this.K - this.F*this.H); 
        }

        if(x_0!=0){
            Bval = Bval + 2*this.A*this.G*x_0;
            Cval = Cval + this.A*(2*this.K + 2*this.H*y_0 + this.G*x_0)*x_0;

            if(m!=0){
                Bval = Bval + 2*this.G*(this.B*m + this.C)*m*x_0;
                Cval = Cval + ((this.C*this.G - this.A*this.H)*x_0 + (2*this.B*this.G)*y_0 + (this.E*this.G - this.D*this.H + this.C*this.K))*m*x_0;
            }
        }

        var sol = solveRootsOfAQuadratic(Aval,Bval,Cval);

        if(!sol[0]){
            print("Root-X1 Error: solution: "+sol)

        }else{// the positive root
            return sol[1]
        }


    }

    this.evalrCrit2X = function(x_0,y_0,m,r_0){
        var J_0 = this.evaluateObj(x_0 + r_0, y_0 + m*r_0);

        var sigma_1 = this.E - this.H*J_0;

        var Aval = this.B*sq(m) + this.C*m + this.A;
        var Bval = 2*this.B*y_0*m + this.C*y_0 + sigma_1*m + this.D - this.G*J_0;
        var Cval = this.B*sq(y_0) + sigma_1*y_0 + this.F - J_0*this.K

        if(x_0!=0){
            Bval = Bval + this.C*m*x_0 + 2*this.A*x_0;
            Cval = Cval + (this.A)*sq(x_0) + (this.C*y_0 + this.D - this.G*J_0)*x_0;
        }

        var sol = solveRootsOfAQuadratic(Aval,Bval,Cval);

        if(!sol[0]){
            // print(this)
            // print("x,y,m,r:"+[x_0,y_0,m,r_0])
            // print(-(this.K*this.D-this.F*this.G)/(this.A*this.K))
            // print((this.K*this.D-this.F*this.G))//grad at 0
            // print(this.F*sq(this.G)-this.D*this.G*this.K +this.A*sq(this.K))//grad2 at 0

            // print(Aval,Bval,Cval)
            if(Math.abs(sol[1])>0.0000001){
                print("Root-X2 Error: solution: "+sol+", J_0:"+J_0+", grad1:"+this.evalGradX(x_0,y_0,m,r_0)+", grad2:"+this.evalGrad2X(x_0,y_0,m))
            }else{
                return 0;
            }
            // gradient has died down though concave!!!
        }else{// the positive root
            return sol[1]
        }
    }



    this.evalGradY = function(x_0,y_0,n,r_0){ // at r=r_0
        //assumed: r_0 = 0; y_0 = 0, n = 0
        var sumVal = 0;
        if(x_0!=0){
            sumVal = sumVal + (this.C*this.G - this.A*this.H)*sq(x_0) + (this.E*this.G - this.D*this.H + this.C*this.K)*x_0;    
        }
        sumVal = sumVal + this.E*this.K - this.F*this.H;

        return sumVal;

    }

    this.evalGrad2Y = function(x_0,y_0,n){
        //assumed: r_0 = 0; y_0 = 0, n = 0
        // all coefs have been divided by two
        var sumVal = 0;
        if(x_0!=0){
            sumVal = sumVal + (this.B*sq(this.G) - this.C*this.G*this.H + this.A*sq(this.H))*sq(x_0);
            sumVal = sumVal + (this.D*sq(this.H) + 2*this.B*this.G*this.K - this.E*this.G*this.H  - this.C*this.H*this.K)*x_0;
        }
        sumVal = sumVal + (this.F*sq(this.H) - this.E*this.H*this.K + this.B*sq(this.K));

        return sumVal;
    }

    this.evalrCrit1Y = function(x_0,y_0,n){
        //assumed: y_0 = 0, n = 0
        
        var Aval = this.B*this.H
        var Bval = 0;
        var Cval = 0;

        if(x_0!=0){
            Bval = Bval + 2*this.B*this.G*x_0;
            Cval = Cval + (this.C*this.G - this.A*this.H)*sq(x_0) + (this.E*this.G - this.D*this.H + this.C*this.K)*x_0;
        }

        Bval = Bval + 2*this.B*this.K;
        Cval = Cval + this.E*this.K - this.F*this.H;

        var sol = solveRootsOfAQuadratic(Aval,Bval,Cval);
        if(!sol[0]){
            print("Root-Y1 Error: solution: "+sol)
        }else{// the positive root
            return sol[1]
        }


    }

    this.evalrCrit2Y = function(x_0,y_0,n,r_0){
        //assumed: y_0 = 0, n = 0, 

        var J_0 = this.evaluateObj(x_0 + n*r_0, y_0+r_0);

        var Aval = this.B;
        var Bval = this.E - this.H*J_0;
        var Cval = this.F - this.K*J_0;

        if(x_0!=0){
            Bval = Bval + this.C*x_0;
            Cval = Cval + (this.A)*sq(x_0) + (this.D - this.G*J_0)*x_0;
        }

        var sol = solveRootsOfAQuadratic(Aval,Bval,Cval);

        if(!sol[0]){
            print("Root-Y2 Error: solution: "+sol)
        }else{// the positive root
            return sol[1]
        }

    }

}
