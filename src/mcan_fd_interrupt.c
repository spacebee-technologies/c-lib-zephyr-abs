/*=============================================================================
 * Author: Spacebeetech - Navegacion
 * Date: 13/01/2023
 * Board: Atmel ARM Cortex-M7 Xplained Ultra Dev Board ATSAMV71-XULT ATSAMV71Q21B
 * Entorno de programacion: Docker with Zephyr
 *
 * Descripcion: Libreria CAN bus en modo fd por interrupcion
 *              Funcionamiento: paso 1: Se configura el periferico can con McanFdInterrupt_configure()
 *                              paso 2: Para enviar mensaje can: Llamar McanFdInterrupt_send(), esta funcion configura el frame a enviar con la informacion indicada y cambia el estado del periferico a APP_STATE_MCAN_TRANSMIT (normalmente esta en APP_STATE_MCAN_USER_INPUT)
 *                                      Para recibir mensaje can: No hacer nada
 *                              paso 3: Cuando se finaliza la transmicion o recepcion, se llama automaticamente a la funcion Callback() correspondeinte la cual cambia el estado de la maquina de estados de can a APP_STATE_MCAN_RECEIVE o  APP_STATE_MCAN_TRANSMIT segun la operacion que se concreto
 *                                      y el estado de can pasa a APP_STATE_MCAN_XFER_SUCCESSFUL si todo salio bien o a APP_STATE_MCAN_XFER_ERROR si algo salio mal. En caso de que sera recepcion, el callback toma el dato recibido y lo almacena en variables globales a esta libreria.
 *                              paso 4: Verificar con una tarea el estado de la maquina de estados de mcan con resultado() y cuando sea APP_STATE_MCAN_XFER_SUCCESSFUL procesar el dato recibido con  McanFdInterrupt_receive().
 *                              Nota: No todos los mensajes can recibidos llaman al callback y guardan el dato. En este caso se estan usando filtros para mensajes SDO de CanOpen, es decir para id que comienza con 0x6XX o 0x5XX. se pueden agregar otros valores en McanFdInterrupt_configure()
 *===========================================================================*/

#include "mcan_fd_interrupt.h"

#include "logger.h"

/*========================================================================
  Funcion: rx_irq_callback
  Descripcion: Callback al cual se llama cuando finalizo una recepcion que pasa el filtro
  Parametro de entrada:
                        const struct device *dev:   Periferico can desde el cual se llamo al callback
                        truct can_frame *frame:     Frame de can que contiene la informacion del mensaje recibido
                        void *user_data             Parametro que indica el usuario para cada filtro añadido
  No retorna nada
  ========================================================================*/
void rx_irq_callback(const struct device *dev, struct can_frame *frame, void *user_data) {
  McanFdInterrupt *self = (McanFdInterrupt *)user_data;

  self->rxMessageLength = can_dlc_to_bytes(frame->dlc);  // Obtengo el tamaño del mensaje can recibido
  self->rxMessageId = frame->id;  // Obtengo el id del mensaje can recibido

  LOG_DEBUG("New received CAN message: ID = 0x%x, Length = 0x%x", (unsigned int)self->rxMessageId, (unsigned int)self->rxMessageLength);

  for (uint8_t loop_count = 0; loop_count < self->rxMessageLength; loop_count++){  // Obtengo mensaje can recibido
    self->rxMessage[loop_count] = frame->data[loop_count];
    LOG_DEBUG("DATA[%u]: 0x%x ", loop_count, self->rxMessage[loop_count]);
  }

  self->xferContext  = APP_STATE_MCAN_RECEIVE;  // Indico que la operacion realizada es recepcion
  self->state = APP_STATE_MCAN_XFER_SUCCESSFUL;  // Indico que se finalizo la recepcion correctamente
}

/*========================================================================
  Funcion: tx_irq_callback
  Descripcion: Callback al cual se llama cuando finalizo una transmision
  Parametro de entrada:
                        const struct device *dev:   Periferico can desde el cual se llamo al callback
                        int error:                  Error que sucede si no se puede enviar. Es cero si se envio correctamente
                        void *arg:                  Argumento de la funcion
  No retorna nada
  ========================================================================*/
void tx_irq_callback(const struct device *dev, int error, void *user_data) {
  McanFdInterrupt *self = (McanFdInterrupt *)user_data;

  if (error != 0) {  // Si existio un error al enviar el mensaje por can
    self->state = APP_STATE_MCAN_XFER_ERROR;  // Cambio estado a mensaje transmitido erroneamente
    LOG_DEBUG("Error sending CAN");
  } else {  // Si se transmitio correctamente
    self->state = APP_STATE_MCAN_XFER_SUCCESSFUL;  // Cambio estado a transmitido correctamente
  }
}

void McanFdInterrupt_new(McanFdInterrupt *self, const struct device *dev) {
  self->xferContext = 0;
  self->state = APP_STATE_MCAN_USER_INPUT;
  self->rxMessageId = 0;
  memset(self->rxMessage, 0, 64);
  self->rxMessageLength = 0;
  self->canDev = dev;
  self->frame.flags = 0;
}

/*========================================================================
  Funcion: McanFdInterrupt_receive
  Descripcion: Recibe mensaje por canbus
  Parametro de entrada:
                        uint32_t *rxMessageId:     Puntero de la variable donde se guardara el Id can del mensaje recibido (de 11 bits/29 bits).
                        uint8_t *rxMessage:        Puntero de la variable donde guardar el mensaje
                        uint8_t *rxMessageLength:  Puntero de la variable donde guardar el tamaÃ±o del mensaje
  Retorna: dato bool indicando si se pudo transmitir el mensaje true o false.
  ========================================================================*/
bool McanFdInterrupt_receive(McanFdInterrupt *self, uint32_t *rxMessageId, uint8_t *rxMessage, uint8_t *rxMessageLength) {
  if (self->state == APP_STATE_MCAN_XFER_SUCCESSFUL && (APP_STATES)self->xferContext == APP_STATE_MCAN_RECEIVE) {
    k_sched_lock();  // Seccion critica para evitar que se ejecute cambio de contexto alterando el proceso de guardado de la variable
    *rxMessageId = self->rxMessageId;
    for (uint8_t i=0; i<self->rxMessageLength; i++) {
      rxMessage[i] = self->rxMessage[i];
    }
    *rxMessageLength = self->rxMessageLength;
    k_sched_unlock();  // Salgo de seccion critica
    return true;  // Retorno falso si se recibio mensaje
  } else {
    return false;  // Retorno falso si no se recibio mensaje
  }
}

/*========================================================================
  Funcion: McanFdInterrupt_configure
  Descripcion: Establece la configuracion del periferico can y configura los filtros de recepcion
  Sin parametro de entrada
  No retorna nada
  ========================================================================*/
void McanFdInterrupt_configure(McanFdInterrupt *self) {
  int ret;  // Variable para indicar el retorno de funciones

  if (!device_is_ready(self->canDev)) {  // Si el dispositivo can no esta listo
    LOG_DEBUG("CAN: Device %s not ready", self->canDev->name);
  }

  can_set_bitrate(self->canDev, 0x7A120);  // Configuro el bitrate de mensaje normal
  can_set_bitrate_data(self->canDev, 0x1E8480);  // Caonfiguro el bitrate del mensaje FD
  ret = can_set_mode(self->canDev, CAN_MODE_FD);  // Activo modo can en modo mensaje normal
  if (ret != 0) {
    LOG_DEBUG("Error setting CAN mode");
  }

  ret = can_start(self->canDev);  // Inicio el periferico CAN
  if (ret != 0) {
    LOG_DEBUG("Error starting CAN controller");
  }

  const struct can_filter filter1 = {  // Estructura que contiene el filtro a aplicara para los mensajes can recibidos, los que cumplen con el filtro llaman al callback rx
    .id = 0x600,   // Base de ID a filtrar
    .mask = 0x600  // Mascara de los bits que deben corresponder para pasar el filtro. 1 debe contener 0 ignora bit
  };

  ret = can_add_rx_filter(self->canDev, &rx_irq_callback, self, &filter1);  //Agrego que contiene la estructura anterior e indico callback al que debe llamar cuando el mensaje recibido pase el filtro.

  if (ret == -ENOSPC) {                       //Si no se puedo agregar el filtro
    LOG_DEBUG("Error, no filter available!");
  }

  const struct can_filter filter2 = {  // Estructura que contiene el filtro a aplicara para los mensajes can recibidos, los que cumplen con el filtro llaman al callback rx
    .id = 0x580,   // Base de ID a filtrar
    .mask = 0x500  // Mascara de los bits que deben corresponder para pasar el filtro. 1 debe contener 0 ignora bit
  };

  ret = can_add_rx_filter(self->canDev, &rx_irq_callback, self, &filter2);   //Agrego que contiene la estructura anterior e indico callback al que debe llamar cuando el mensaje recibido pase el filtro.

  if (ret == -ENOSPC) {                       //Si no se puedo agregar el filtro
    LOG_DEBUG("Error, no filter available!");
  }
  LOG_DEBUG("Finished init");
}

/*========================================================================
  Funcion: McanFdInterrupt_send
  Descripcion: Envia mensaje por canbus
  Parametro de entrada:
                        uint32_t messageID:     Id can a donde enviar el mensaje de 11 bits/29 bits.
                        uint8_t *message:       Puntero del mensaje a enviar
                        uint8_t messageLength:  TamaÃ±o del arreglo a enviar
                        MCAN_MODE MCAN_MODE:    Modo de operacion can
                                                Mensajes FD (can FD- alta velocidad) con id estandar:                   MCAN_MODE_FD_STANDARD
                                                Mensajes FD (can FD- alta velocidad) con id extendido:                  MCAN_MODE_FD_EXTENDED
                                                Mensajes estandar normal (can clasico) hasta 8 bytes:                   MCAN_MODE_NORMAL
  Retorna: dato bool indicando si se pudo transmitir el mensaje true o false.
  Nota: los mensajes FD puedene enviar de 0 a 64 bytes, mientras que los menajes normales pueden enviar de 0 a 8 bytes.
  ========================================================================*/
bool McanFdInterrupt_send(McanFdInterrupt *self, uint32_t messageId, uint8_t *message, uint8_t messageLength, MCAN_MODE mcanMode) {
  if (self->state == APP_STATE_MCAN_USER_INPUT) {

    self->frame.flags = 0;
    if (mcanMode == MCAN_MODE_FD_STANDARD || mcanMode == MCAN_MODE_FD_EXTENDED) {
      self->frame.flags |= CAN_FRAME_FDF;  // TODO: Check if we can support also BRS
    }
    if (mcanMode == MCAN_MODE_FD_EXTENDED || mcanMode == MCAN_MODE_EXTENDED) {
      self->frame.flags |= CAN_FRAME_IDE;
    }

    self->frame.id = messageId;

    for (uint8_t loop_count = 0; loop_count < messageLength; loop_count++) {
      self->frame.data[loop_count] = message[loop_count];
    }

    self->frame.dlc = can_bytes_to_dlc(messageLength);
    /* This sending call is none blocking. */
    uint8_t res = can_send(self->canDev, &self->frame, K_FOREVER, tx_irq_callback, self);
    /* This sending call is blocking until the message is sent. */
    //can_send(canDev, &frame, K_MSEC(100), NULL, NULL);
    if (res != 0) {
      return false;  // Retorno falso si no se pudo enviar
    }
    self->state = APP_STATE_MCAN_TRANSMIT;
    self->xferContext = APP_STATE_MCAN_TRANSMIT;
    return true;
  } else {
    return false;  // Retorno falso si no se pudo enviar
  }
}

/*========================================================================
  Funcion: McanFdInterrupt_getState()
  Descripcion: Indica el resultado del can bus luego de llamar McanFdInterrupt_send() o McanFdInterrupt_receive() para determina si un mensaje se envio o recibio correctamente.
               Esta pensado para llamar a esta funcion continuamente en una tarea para determinar si hay un nuevo dato recibido correctamente luego de llamar McanFdInterrupt_receive().
               Esta pensado para llamar a esta funcion continuamente en una tarea para determinar si se envio correctamente el mensaje can luego de llamar a la funcion McanFdInterrupt_send().
  Sin parameto de entrada.
  Retorna: uint8_t estado: indica el estado del can bus:
                        0 = no se esta esperando una transmision o recepcion
                        1 = Se recibio correctamente un dato por can bus luego de llamar a la funcion McanFdInterrupt_receive()
                        2 = Se transmitio correctamente un dato por can bus luego de llamar a la funcion McanFdInterrupt_send()
                        3 = Error al recibir un dato por can bus luego de llamar a la funcion McanFdInterrupt_receive()
                        4 = Error al transmitir un dato por can bus luego de llamar a la funcion McanFdInterrupt_send()
  ========================================================================*/
uint8_t McanFdInterrupt_getState(McanFdInterrupt *self) {
  uint8_t resultado = 0;
  switch (self->state) {
    case APP_STATE_MCAN_XFER_SUCCESSFUL:  // Si la transmicion o recepcion se realizo con exito
    {
      if ((APP_STATES)self->xferContext == APP_STATE_MCAN_RECEIVE)  // Si el contexto era de recepcion
      {
        resultado = 1;  // Se recibio correctamente
      } else if ((APP_STATES)self->xferContext == APP_STATE_MCAN_TRANSMIT) {
        resultado = 2;  // Se transmitio correctamente
      }
      break;
    }
    case APP_STATE_MCAN_XFER_ERROR:  // Si la transmicion o recepcion fue erronea
    {
      if ((APP_STATES)self->xferContext == APP_STATE_MCAN_RECEIVE)  // Si el contecto era de recepcion
      {
        resultado = 3;  // Error al recibir mensaje
      } else {
        resultado = 4;  // Error al enviar mensaje
      }
      break;
    }
    default:  // En cualquier otro estado del can
      break;
    }
    return resultado;
}

/*========================================================================
  Funcion: McanFdInterrupt_enable()
  Descripcion: Establece el estado de la maquina de estado de can en user_input para poder configurar una nueva transmicion o recepcion de datos can
               Recordar que las funciones McanFdInterrupt_receive() y McanFdInterrupt_send() no se ejecutan si el estado de la maquina no es  APP_STATE_MCAN_USER_INPUT.
               Con esto se evita que si todavia no se tomo el dato que llego por can, no se sobreescriba.
  Sin parameto de entrada.
  No retorna nada
  ========================================================================*/
void McanFdInterrupt_enable(McanFdInterrupt *self) {
  self->state = APP_STATE_MCAN_USER_INPUT;
}
