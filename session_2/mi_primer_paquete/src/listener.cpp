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

/**
 * Función de devolución de llamada (callback) del tópico `chatter`.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("Escuche: [%s]", msg->data.c_str());
}

/**
 * Este ejemplo demuestra la recepción simple de mensajes en el sistema ROS.
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
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle es el principal punto de acceso a las comunicaciones con el
   * sistema ROS. El primer NodeHandle construido inicializará completamente
   * este nodo y el último NodeHandle destruido finalizará el nodo.
   */
  ros::NodeHandle nh;

  /**
   * La llamada subscribe() es la forma de informarle a ROS que querés recibir
   * mensajes sobre un tópico determinado. Esto invoca una llamada al ROS
   * master, que mantiene un registro de quién está publicando y quién se está
   * suscribiendo. Los mensajes se pasan a una función de devolución de llamada
   * (callback), aquí llamada chatterCallback. subscribe() devuelve un objeto
   * Suscriptor que debe ser conservado hasta que se desee desuscribirse de
   * dicho tópico. Cuando todas las copias del el objeto de suscriptor quedan
   * fuera de scope, el callback quedará automáticamente desuscripto de dicho
   * tópico.

   * El segundo parámetro de la función subscribe() es el tamaño de la cola de
   * mensajes. Si los mensajes llegan más rápido de lo que se procesan, esto es
   * el número de mensajes que se almacenarán en el búfer antes de comenzar a
   * tirar los más viejos.
   */
  ros::Subscriber chatter_sub = nh.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() entrará en un bucle, llamando a los callbacks. En este nodo,
   * todos los callbacks se llamarán desde este hilo (el principal). ros::spin()
   * terminará su ejecución cuando se presione Ctrl-C, o el nodo sea finalizado
   * por el ROS master.
   */
  ros::spin();

  return 0;
}
