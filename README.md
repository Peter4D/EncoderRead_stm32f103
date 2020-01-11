# EncoderRead_stm32f103
Read out quadrature encoder data and calculate speed acceleration and direction of rotation
Code utilise of hardware timer 1 encoder read mode (automaticaly increment or decrement timer value depend
on rotation direction of encoder)

End application requirements is to detect the point of deceleration and after some time
activate digital output to signal that deceleration is detected. 
