#include "Compress_States.h"


void compressStates(){
    StatesZ_CTRL.xy = compressXY(statePos.x,statePos.y);
    StatesZ_CTRL.z = (int16_t)(statePos.z * 1000.0f);

    StatesZ_CTRL.vxy = compressXY(stateVel.x, stateVel.y);
    StatesZ_CTRL.vz = (int16_t)(stateVel.z * 1000.0f);

    StatesZ_CTRL.wxy = compressXY(stateOmega.x/10,stateOmega.y/10);
    StatesZ_CTRL.wz = (int16_t)(stateOmega.z * 1000.0f);


    float const q[4] = {
        stateQuat.x,
        stateQuat.y,
        stateQuat.z,
        stateQuat.w};
    StatesZ_CTRL.quat = quatcompress(q);

    // COMPRESS SENSORY VALUES
    StatesZ_CTRL.Theta_xy = compressXY(Theta_x,Theta_y);
    StatesZ_CTRL.Tau = (int16_t)(Tau * 1000.0f); 
    StatesZ_CTRL.D_perp = (int16_t)(D_perp * 1000.0f);

    // COMPRESS THRUST/MOMENT VALUES
    StatesZ_CTRL.FMz = compressXY(F_thrust,M.z*1000.0f);
    StatesZ_CTRL.Mxy = compressXY(M.x*1000.0f,M.y*1000.0f);

    // COMPRESS MOTOR THRUST VALUES
    StatesZ_CTRL.M_thrust12 = compressXY(M1_thrust,M2_thrust);
    StatesZ_CTRL.M_thrust34 = compressXY(M3_thrust,M4_thrust);

    
    // COMPRESS PWM VALUES
    StatesZ_CTRL.MS_PWM12 = compressXY(M1_pwm*0.5e-3f,M2_pwm*0.5e-3f);
    StatesZ_CTRL.MS_PWM34 = compressXY(M3_pwm*0.5e-3f,M4_pwm*0.5e-3f);

    StatesZ_CTRL.Policy_Trg_Action = (int16_t)(Policy_Trg_Action * 1000.0f);
    StatesZ_CTRL.Policy_Flip_Action = (int16_t)(Policy_Flip_Action * 1000.0f);

}

void compressSetpoints(){
    setpointZ_CTRL.xy = compressXY(x_d.x,x_d.y);
    setpointZ_CTRL.z = (int16_t)(x_d.z * 1000.0f);

    setpointZ_CTRL.vxy = compressXY(v_d.x,v_d.y);
    setpointZ_CTRL.vz = (int16_t)(v_d.z * 1000.0f);

    setpointZ_CTRL.axy = compressXY(a_d.x,a_d.y);
    setpointZ_CTRL.az = (int16_t)(a_d.z * 1000.0f);
}

void compressFlipStates(){
    FlipStatesZ_CTRL.xy = compressXY(statePos_tr.x,statePos_tr.y);
    FlipStatesZ_CTRL.z = (int16_t)(statePos_tr.z * 1000.0f);

    FlipStatesZ_CTRL.vxy = compressXY(stateVel_tr.x, stateVel_tr.y);
    FlipStatesZ_CTRL.vz = (int16_t)(stateVel_tr.z * 1000.0f);

    FlipStatesZ_CTRL.wxy = compressXY(stateOmega_tr.x,stateOmega_tr.y);
    FlipStatesZ_CTRL.wz = (int16_t)(stateOmega_tr.z * 1000.0f);


    float const q[4] = {
        stateQuat_tr.x,
        stateQuat_tr.y,
        stateQuat_tr.z,
        stateQuat_tr.w};
    FlipStatesZ_CTRL.quat = quatcompress(q);

   FlipStatesZ_CTRL.Theta_xy = compressXY(Theta_x_tr,Theta_y_tr);
   FlipStatesZ_CTRL.Tau = (int16_t)(Tau_tr * 1000.0f); 
   FlipStatesZ_CTRL.D_perp = (int16_t)(D_perp_tr * 1000.0f);

   FlipStatesZ_CTRL.NN_FP = compressXY(Policy_Flip_tr,Policy_Action_tr); // Flip value (OC_SVM) and Flip action (NN)


}


uint32_t compressXY(float x, float y)
{ 
  
  uint16_t xnew, ynew;
  uint32_t xy;

  // CONVERT FLOATS TO INTS OFFSET BY UINT16_MAX/2.0
  xnew = x*1000.0f + 32767.0f;
  ynew = y*1000.0f + 32767.0f;


  // CLIP RANGES OF VALUES
  xnew = (xnew < UINT16_MAX) ? xnew : UINT16_MAX;
  xnew = (xnew > 0) ? xnew : 0;

  ynew = (ynew < UINT16_MAX) ? ynew : UINT16_MAX;
  ynew = (ynew > 0) ? ynew : 0;

  // APPEND YNEW BYTES TO XNEW BYTES
  xy = (xnew << 16 | ynew); // Shift xnew by 16 and combine

  return xy;
};
