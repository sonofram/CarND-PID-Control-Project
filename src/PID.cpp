#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/



PID::PID(int window_size, int max_steps,double err_tolerance) {
	PID::window_size = window_size;
	PID::max_steps = max_steps;
	PID::err_tolerance = err_tolerance;

	PID::steps = 1;

	PID::dp[0] = 0.1;
	PID::dp[1] = 0.0001;
	PID::dp[2] = 1;

	//PID::dp[0] = 1;
	//PID::dp[1] = 1;
	//PID::dp[2] = 1;


	PID::Kp = 0.0;
	PID::Ki = 0.0;
	PID::Kd = 0.0;

	PID::i_error = 0.0;
	PID::d_error = 0.0;
	PID::p_error = 0.0;

	PID::err = 0.0;

}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	PID::Kp = Kp;
	PID::Ki = Ki;
	PID::Kd = Kd;

}

void PID::Twiddle(double cte){

	if(PID::steps == 1){
		PID::best_err = cte;
	}

	PID::err += cte*cte;

	if(PID::best_err < PID::err_tolerance){
			if(steps%window_size == 0){
				PID::err = (PID::err/PID::window_size);
				PID::best_err = PID::err;
				PID::err = 0;
			}
			PID::steps += 1;
			return;
	}



	//if(steps%window_size != 0){
	//	return;
	//}else{
	//	cout << "steps: " << PID::steps;
	//}


	 /***********************Calculating for Kp******************/

	//Setting up to calculate best error
	if(PID::steps == PID::window_size){
		std::cout << "window 1 ************************" << std::endl;
        std::cout << "Kp,Ki,Kd: " << PID::Kp << "," << PID::Ki << "," << PID::Kd << " best_err: " << PID::best_err << std::endl;

        PID::err = (PID::err/PID::window_size);
		//Get the Error calculated for first $window_size steps.
		PID::best_err = PID::err;
		//Add probe value.
		PID::Kp += dp[0];
		//Reset error to calculate next $window_size steps.
		PID::err = 0.0;

	}else if(PID::steps == 2*PID::window_size){
		std::cout << "window 2 ************************" << std::endl;
        std::cout << "Kp,Ki,Kd: " << PID::Kp << "," << PID::Ki << "," << PID::Kd << " best_err: " << PID::best_err << std::endl;
        PID::err = (PID::err/PID::window_size);
		//Based on err calculated adjust the probe value.
		if(PID::err < PID::best_err){
			PID::best_err = PID::err;
			PID::dp[0] *= 1.1;
		}else{
			PID::Kp -= 2*dp[0];
			//PID::Kp -= dp[0];
			PID::kp_decrement = true;
		}

		//Reset error to calculate next $window_size steps.
		PID::err = 0.0;

	}else if(PID::steps == 3*PID::window_size){
		std::cout << "window 3 ************************" << std::endl;
        std::cout << "Kp,Ki,Kd: " << PID::Kp << "," << PID::Ki << "," << PID::Kd << " best_err: " << PID::best_err << std::endl;
        PID::err = (PID::err/PID::window_size);

        if(PID::kp_decrement){
			if(PID::err < PID::best_err){
				PID::best_err = PID::err;
				dp[0] *= 1.1;
			}else{
				PID::p_error += dp[0];
				dp[0] *= 0.9;
			}
        }//if(PID::kp_decrement)
        /***********************Calculating for Ki******************/
        else{
    		PID::best_err = PID::err;
        	//Add probe value.
    		PID::Ki += dp[1];
        }
		//Reset error to calculate next $window_size steps.
		PID::err = 0.0;

	}else if(PID::steps == 4*PID::window_size){
		std::cout << "window 4 ************************" << std::endl;
        std::cout << "Kp,Ki,Kd: " << PID::Kp << "," << PID::Ki << "," << PID::Kd << " best_err: " << PID::best_err << std::endl;
        PID::err = (PID::err/PID::window_size);
		//Based on err calculated adjust the probe value.
		if(PID::err < PID::best_err){
			PID::best_err = PID::err;
			PID::dp[1] *= 1.1;
		}else{
			PID::Ki -= 2*dp[1];
			//PID::Ki -= dp[1];
			PID::ki_decrement = true;
		}

		//Reset error to calculate next $window_size steps.
		PID::err = 0.0;

	}else if(PID::steps == 5*PID::window_size){
		std::cout << "window 5 ************************" << std::endl;
        std::cout << "Kp,Ki,Kd: " << PID::Kp << "," << PID::Ki << "," << PID::Kd << " best_err: " << PID::best_err << std::endl;
        PID::err = (PID::err/PID::window_size);
        if(PID::ki_decrement){
			if(PID::err < PID::best_err){
				PID::best_err = PID::err;
				dp[1] *= 1.1;
			}else{
				PID::Ki += dp[1];
				dp[1] *= 0.9;
			}
        }//if(PID::ki_decrement){
        /***********************Calculating for Kd******************/
        else{
    		//Get the Error calculated for first $window_size steps.
    		PID::best_err = PID::err;
    		//Add probe value.
    		PID::Kd += dp[2];
        }
		//Reset error to calculate next $window_size steps.
		PID::err = 0.0;

	}else if(PID::steps == 6*PID::window_size){
		std::cout << "window 6 ************************" << std::endl;
        std::cout << "Kp,Ki,Kd: " << PID::Kp << "," << PID::Ki << "," << PID::Kd << " best_err: " << PID::best_err << std::endl;
        PID::err = (PID::err/PID::window_size);
		//Based on err calculated adjust the probe value.
		if(PID::err < PID::best_err){
			PID::best_err = PID::err;
			PID::dp[2] *= 1.1;
		}else{
			PID::Kd -= 2*dp[2];
			//PID::Kd -= dp[2];
			PID::kd_decrement = true;
		}

		//Reset error to calculate next $window_size steps.
		PID::err = 0.0;

	}else if(PID::steps == 7*PID::window_size){

		std::cout << "window 7 ************************" << std::endl;
        std::cout << "Kp,Ki,Kd: " << PID::Kp << "," << PID::Ki << "," << PID::Kd << " best_err: " << PID::best_err << std::endl;
        PID::err = (PID::err/PID::window_size);
        if(PID::kd_decrement){
			if(PID::err < PID::best_err){
				PID::best_err = PID::err;
				dp[2] *= 1.1;
			}else{
				PID::Kd += dp[2];
				dp[2] *= 0.9;
			}
        }

		//Reset error to calculate next $window_size steps.
		PID::err = 0.0;

		//Resetting steps to repeat.
		PID::steps = 1;
	}

	PID::steps += 1;

}//Twiddle


void PID::UpdateError(double cte) {

    //if(PID::steps == 1){
  	//  PID::prev_cte = cte;
    //}

	PID::p_error = cte;
	PID::d_error = cte - PID::prev_cte;
	PID::i_error = PID::i_error + cte;

	PID::prev_cte = cte;
}

double PID::TotalError() {
	return -Kp * p_error - Kd * d_error - Ki * i_error;
}

