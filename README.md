# avr_moto
#### h4 Leds for motorbike 2016

Плата имеет защиту от переполюсовки клемм АБ на входе; 
Цепи питания АБ и цепи питания MCU гальванически развязанны Ирбисом.
DC/DC преобразователь защищен самовостанавливающимся предохранителем.
---
Плата детектирует
   -срабатывания концевика педали тормоза
   -включения левого поворотника 
   -включения правого поротника
Эти входы на схеме также оптически развязанны от цепей питания байка.
---
Плата по прерыванию выходит из спящего режима и запускает соответсвущ. ф-ю: стробирование стоп-сигнала, бегущий огонь поворотников.
Все светодиодные группы управляются LED-драйверами для защиты LED от экстреальных режимов работы (высокого тока, импульсов напряжения до 50В, температуры).  
---
С помощью DIP-переключателя на плате:
   -переключать строб сигнал стоп-огней на непрерывный (switch 4);
   -менять режимы бегущего огня поворников (всего 8 режимов). 
---
На плате 6 контактных площадок для подключения:
-светод. групп светодиодов стоп-сигнала и габаритов 
-светод. групп лев. поворотника к LED-драйверам;
-светод. групп прав. поворотника к LED-драйверам;
-cигнала тормоза, сигналов поворотника
-минуса АБ (-Ubat)  
-плюса АБ  (+Ubat)
