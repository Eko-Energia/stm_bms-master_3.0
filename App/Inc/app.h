#ifndef APP_H
#define APP_H

/** @brief Application entry point. Called from main(), never returns. */
void app_main(void);

/** @brief Safe shutdown for Error_Handler: open the contactor, annunciate. */
void App_OnFatalError(void);

#endif /* APP_H */
