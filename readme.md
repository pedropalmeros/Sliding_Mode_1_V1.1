In this code a First Order Sliding Mode is presented. The Sliding Surface is given by

sigma = omega_e + beta dot_qe

but the controller is slow although all gains are big enough, hence the next step is to modify the 
sliding surface as 

sigma = alpha*omega_e + beta* dot_qe

This change has to be done theoretically because surely it will modify the entire control law
