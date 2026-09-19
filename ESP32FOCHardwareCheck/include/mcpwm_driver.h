#pragma once

/** Hook ADC capture onto SimpleFOC's existing MCPWM (timer0 TEP = PWM center). */
void mcpwmAttachAdcIsr();
void mcpwmPauseAdcIsr();
void mcpwmResumeAdcIsr();
