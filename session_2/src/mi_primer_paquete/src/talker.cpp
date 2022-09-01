/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the
 * names of its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

/**
 * Este ejemplo demuestra el envío simple de mensajes a través del sistema ROS.
 */
int main(int argc, char** argv) {
  /**
   * La función ros::init() necesita recibir argc y argv para que distintas
   * funcionalidades de ROS puedan operar (tales como el remapeo de nombres).
   * El tercer argumento es el nombre del nodo.
   *
   * Se debe llamar a ros::init() antes de usar cualquier otra parte del sistema
   * ROS.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle es el principal punto de acceso a las comunicaciones con el
   * sistema ROS. El primer NodeHandle construido inicializará completamente
   * este nodo y el último NodeHandle destruido finalizará el nodo.
   */
  ros::NodeHandle nh;

  /**
   * La función advertise() es la forma de informarle a ROS que querés publicar
   * sobre un tópico determinado. Esto invoca una llamada al ROS master, que
   * mantiene un registro de quién está publicando y quién se está suscribiendo.
   * Después de realizar esta llamada de advertise(), el ROS master notificará a
   * cualquiera que intente suscribirse a este tópico, y ellos a su vez
   * negociarán una conexión peer-to-peer con este nodo. advertise() devuelve un
   * objeto Publisher que te permite publicar mensajes en este tópico a través
   * de una llamada a la publish(). Una vez que todas las copias del objeto
   * Publisher devuelto son destruídas, el tópico se anulará automáticamente.

   * El segundo parámetro de advertise() es el tamaño de la cola de mensajes
   * utilizado para publicar mensajes. Si los mensajes se publican más
   * rápidamente de lo que podemos enviarlos, el número aquí especifica cuántos
   * mensajes acumular antes de comenzar a eliminarlos.
   */
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * El número de mensajes que hemos enviado. Esto se usa para crear un string
   * único para cada mensaje.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * Este es un objeto de mensaje. Lo rellenás con datos y luego lo publicás.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hola mundo " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * La función publish() es la forma en que enviás mensajes. El parámetro es
     * el objeto del mensaje. El tipo de este objeto debe coincidir con el tipo
     * dado como un parámetro del template para la llamada de advertise<>(),
     * como se hizo en el constructor de arriba.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
