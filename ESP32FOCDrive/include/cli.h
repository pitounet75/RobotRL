#pragma once

/** Serial command line, core 1 only. Never called from the FOC task. */
void cliInit();
void cliPoll();
void cliPrintHelp();
