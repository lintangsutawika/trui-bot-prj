#include "sc.h"


namespace trui {

  Sc::Sc (size_t pwm_pin_, size_t dir_pin1_,size_t dir_pin2_, size_t encoder_out_a_pin, size_t encoder_out_b_pin, uint16_t encoder_resolution, float outmax_, float outmin_) : pwm_pin_(pwm_pin_), dir_pin1_(dir_pin1_), dir_pin2_(dir_pin2_), encoder_out_a_pin(encoder_out_a_pin), encoder_out_b_pin(encoder_out_b_pin), encoder_resolution(encoder_resolution), outmax_(outmax_), outmin_(outmin_){

  

  // Motor::Motor (size_t pwm_pin_, size_t dir_pin1_, size_t dir_pin2_,/* size_t en_1_pin, size_t en_2_pin, size_t xy_pin,*/ size_t oe_pin, size_t sel_1_pin, size_t sel_2_pin, size_t reset_x_pin,/* size_t reset_y_pin,*/ size_t bit0, size_t bit1,size_t bit2, size_t bit3, size_t bit4, size_t bit5, size_t bit6, size_t bit7, float outmax_, float outmin_) : pwm_pin_(pwm_pin_), dir_pin1_(dir_pin1_), dir_pin2_(dir_pin2_), /*en_1_pin(en_1_pin), en_2_pin(en_2_pin), xy_pin(xy_pin),*/ oe_pin(oe_pin), sel_1_pin(sel_1_pin), sel_2_pin(sel_2_pin), reset_x_pin(reset_x_pin), /*reset_y_pin(reset_y_pin),*/ bit0(bit0) , bit1(bit1) , bit2(bit2) , bit3(bit3), bit4(bit4) , bit5(bit5) , bit6(bit6) , bit7(bit7), outmax_(outmax_), outmin_(outmin_){
    omega_=0,omega_read_=0, omega_input_=0, last_omega_=0;
    omega_read_k_1_=0;
    omega_read_k_2_ =0;
    omega_read_k_3_ =0;
    omega_read_k_4_ =0;
    omega_read_refined = 0;
    tick_= 0;
    tick_enc_= 0;
    last_tick_enc_= 0;
    last2_tick_enc_= 0; 
    mv_=0, iTerm_= 0;    
    delta_=0, error_=0, last_error_=0; 
    data_=0;
    // kp_= 0.4, ki_= 0.05, kd_= 0.02;
    // kp_= 0.4, ki_= 0.05, kd_= 5;
    // kp_= 1, ki_= 0.05, kd_= 0.1;
    kp_= 4, ki_= 0.15, kd_= 3;
    // kp_= 1.5, ki_= 0.05, kd_= 0.1;
    // kp_= 0.5, ki_= 0.0528, kd_= 0;//for 50ms sampling time
    setup();

    encoder_ = new crim::TwoPhaseIncrementalEncoder(encoder_out_a_pin, encoder_out_b_pin, encoder_resolution); 

  }

  Sc::~Sc() {
    delete encoder_;
  }



  void Sc::setup() {
    pinMode(dir_pin1_, OUTPUT);
    pinMode(dir_pin2_, OUTPUT);
    pinMode(pwm_pin_, OUTPUT);   
    encoder_->reset_Enc();
    // TCCR2B = TCCR2B & 0b11111000 | 0x05; 
  }

  void Sc::reset(){
    encoder_->reset_Enc();
  }

  int64_t Sc::read_encoder(){
    tick_enc_ = encoder_->pos();
    omega_ = (float)(tick_enc_ - last_tick_enc_)*1500.0/133.0;//57.0//3000.0/449;// Tetha = ((tickEnc - last_tickEnc)/57) * 2 * PI rad
                                             // omega = Tetha / Delta_Time -- Delta_Time = 10ms
                                             // omega = Tetha / 10ms = ((tickEnc - last_tickEnc)/57) * 2 * PI * 1000/10 rad/s
                                             // omega = (tickEnc - last_tickEnc) * 200/57 * PI rad/s
                                             // omega = (tickEnc - last_tickEnc) * 200/57 * PI * (1/(2PI)) rotation/rad rad/s
                                             // omega = (tickEnc - last_tickEnc) * 100/57 rotation/s
                                             // omega = (tickEnc - last_tickEnc) * 100/57 rotation/(1/60) minute
                                             // omega = (tickEnc - last_tickEnc) * 100/57 * 60 rotation/minute
                                             // omega = (tickEnc - last_tickEnc) * 6000/57 RPM
                                             // Quadrature -> 1500/57 RPM
                                             // for 13ppr -> 3000/449
    last_tick_enc_ = tick_enc_;  
    return omega_;
  }

  int64_t Sc::get_encoder(){
    tick_enc_ = encoder_->pos();
    return encoder_->pos();
  }

  int64_t Sc::get_pos(){ //Special method just for recording. gets data from set_speed method
    int64_t enc_pos_ = tick_enc_;
    return enc_pos_;
  }

  void Sc::testing_encoder(){
    while (true) {
      
      Serial.println("===");
      // Serial.println(encoder_->readByte());
      Serial.println(static_cast<long int>(encoder_->pos()));
      // Serial.println(encoder_->rot());
      delay(100);
    }
  }

  void Sc::PIDvelocity_algorithm(float speed,float Kp,float Ki,float Kd,float delta_T){
      omega_input_ = speed; //setpoint
      tick_enc_ = encoder_->pos();
      omega_ = (float)(tick_enc_ - last_tick_enc_)*600/449;//300/57;// Tetha = ((tickEnc - last_tickEnc)/228) * 2 * PI rad
                                             // omega = Tetha / Delta_Time -- Delta_Time = 50ms
                                             // omega = Tetha / 50ms = ((tickEnc - last_tickEnc)/228) * 2 * PI * 1000/50 rad/s
                                             // omega = (tickEnc - last_tickEnc) * 40/228 * PI rad/s
                                             // omega = (tickEnc - last_tickEnc) * 40/228 * PI * (1/(2PI)) rotation/rad rad/s
                                             // omega = (tickEnc - last_tickEnc) * 20/228 rotation/s
                                             // omega = (tickEnc - last_tickEnc) * 20/228 rotation/(1/60) minute
                                             // omega = (tickEnc - last_tickEnc) * 5/57 * 60 rotation/minute
                                             // omega = (tickEnc - last_tickEnc) * 300/57 RPM
                                             // Quadrature -> 300/57 RPM
                                             // for 13ppr -> 600/449
      e_k_ = omega_input_ - omega_;
      U_t_ = U_t_1_ + (Kp+Ki*delta_T+Kd/delta_T)*e_k_-(Kp+2*Kd/delta_T)*e_k_1_+(Kd/delta_T)*e_k_2_;
      e_k_2_ = e_k_1_;
      e_k_1_ = e_k_;
      U_t_1_ = U_t_;
      last_tick_enc_ = tick_enc_;  
      outSignal(U_t_);
      //return U_t_;

  }

  float Sc::set_speed(float cmd_speed) {
    omega_input_ = cmd_speed; //setpoint
      // tick_enc_ = encoder_->pos();
      omega_read_k_ = read_encoder();
      omega_read_refined = (omega_read_k_ + omega_read_k_1_ + omega_read_k_2_ + omega_read_k_3_ + omega_read_k_4_)/5;
      error_ = omega_input_ - omega_read_refined;
      iTerm_ = iTerm_ + (float)error_*ki_;       //Integral Term of PID Control 
      
      if(iTerm_ > outmax_) iTerm_ = outmax_;
      else if(iTerm_ < outmin_) iTerm_ = outmin_;
                    
      deriv_comp_ = kd_* (last_error_ - error_);//error_ - deriv_comp_;//(tick_enc_ - 2*last_tick_enc_ + last2_tick_enc_);//*numerator_/denominator_;
                    
      mv_ =  (float)error_*kp_ + iTerm_ + deriv_comp_;
      
 

      if((cmd_speed > 0 && mv_ < 0) || (cmd_speed < 0 && mv_ > 0)) mv_ = 0;//outmax_;
      else if(cmd_speed > 0 && mv_ > 255) mv_ = 255;
      else if(cmd_speed < 0 && mv_ < -255) mv_ = -255;
      else if (cmd_speed == 0) mv_ = 0;
                    
      outSignal(mv_); 
      omega_read_k_4_ = omega_read_k_3_;
      omega_read_k_3_ = omega_read_k_2_;
      omega_read_k_2_ = omega_read_k_;
      last_error_ = error_;
      return omega_read_refined;
      // last2_tick_enc_ = last_tick_enc_; 
      // last_tick_enc_ = tick_enc_;    
  }

  void Sc::outSignal(float pwm){
  //COUNTER CLOCK WISE IS MINUS TICKING
  //PLUS SIGNEDPWM IS CLOCKWISE
      if(pwm>255) pwm = 255;
      else if(pwm<-255) pwm = -255;
      
      ocr_ = (float)pwm;
      
      if(pwm<0) {
        digitalWrite(dir_pin1_, LOW);
        digitalWrite(dir_pin2_, HIGH);
        analogWrite(pwm_pin_, -ocr_);
      }
      else if(pwm>0) {
        digitalWrite(dir_pin1_, HIGH);
        digitalWrite(dir_pin2_, LOW);
        analogWrite(pwm_pin_, ocr_);
      }
      else if (pwm == 0) {
        digitalWrite(dir_pin1_, HIGH);
        digitalWrite(dir_pin2_, HIGH);
        analogWrite(pwm_pin_, 0);
      }
  }

}// namespace trui
