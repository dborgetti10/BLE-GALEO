# BLE-GALEO
USING ESP 32 as a server to communicate with an already made android app for access control

/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleWrite.cpp
    Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

BLECharacteristic *pK_Soft_Rev;
BLECharacteristic *pK_Relay_2;
BLECharacteristic *pK_Relay_1;
BLECharacteristic *pK_Master_Code;
BLECharacteristic *pK_Buzzer_Key;



 bool deviceConnected = false;
float txValue = 0;
const int readPin = 27; // Use GPIO number. See ESP32 board pinouts
const int LED = 16; //

//#ifndef __ESP_GATT_DEFS_H__
#define __ESP_GATT_DEFS_H__

#include "esp_bt_defs.h"
          
#define ESP_GATT_ATTR_HANDLE_MAX            100
#define ESP_GATT_MAX_READ_MULTI_HANDLES     10           /* Max attributes to read in one request */


//GALEO
#define SERV_UUID_180A                     "0000180a-0000-1000-8000-00805f9b34fb"    //DEVICE_INFORMATION_SERVICE
  #define K_MANUF_NAME_UUID_2A29           "00002a29-0000-1000-8000-00805f9b34fb"      //Manufacture Name    
  #define K_MOD_NB_UUID_2A24               "00002a24-0000-1000-8000-00805f9b34fb"      //Model Number
  #define K_HARD_REV_UUID_2A27             "00002a27-0000-1000-8000-00805f9b34fb"      //Hardware revision
  #define K_FIRM_REV_UUID_2A26             "00002a26-0000-1000-8000-00805f9b34fb"      //Firmware Revision
  #define K_SOFT_REV_UUID_2A28             "00002a28-0000-1000-8000-00805f9b34fb"      //Software Revision
  #define K_SYST_ID_UUID_2A23              "00002a23-0000-1000-8000-00805f9b34fb"      //System ID
  #define K_PNP_ID_UUID_2A50               "00002a50-0000-1000-8000-00805f9b34fb"      //Pnp ID
  #define K_SERIAL_NUM_UUID_2A25               "00002a25-0000-1000-8000-00805f9b34fb"      //SERIAL_NUMBER_STR

#define SERV_UUID_190A                     "0000190a-0000-1000-8000-00805f9b34fb"    //SERVICE contrôle d’accès
  #define CHARACTERISTIC_UUID_2931         "00002931-0000-1000-8000-00805f9b34fb"    // Caractéristique Status Door
   #define CHARACTERISTIC_UUID_292F         "0000292f-0000-1000-8000-00805f9b34fb"   // Caractéristique Relay States

#define SERV_UUID_55AA                     "000055aa-0000-1000-8000-00805f9b34fb"    //SERVICE CRYPTO
  #define CHARACTERISTIC_UUID_55AA         "000055aa-0000-1000-8000-00805f9b34fb"   // Caractéristique CRYPTO
  #define CHARACTERISTIC_UUID_55AAA        "000055aa-0000-0009-0000-805f9b34fb00"   // Caractéristique CRYPTO2

#define SERV_UUID_190B                     "0000190b-0000-1000-8000-00805f9b34fb"    //SERVICE Configurations
#define SERV_UUID_190BB                    "0000190b-0000-1000-8000-00805f9b34fb"    //SERVICE Configurations
  #define CHARACTERISTIC_UUID_2932         "00002932-0000-1000-8000-00805f9b34fb"    // Caractéristique Code Relay 1
  #define CHARACTERISTIC_UUID_2936         "00002936-0000-1000-8000-00805f9b34fb"    // Caractéristique Code Relay 1 suite
  #define CHARACTERISTIC_UUID_2933         "00002933-0000-1000-8000-00805f9b34fb"    // Caractéristique Code Relay 2
  #define CHARACTERISTIC_UUID_2934         "00002934-0000-1000-8000-00805f9b34fb"    // Caractéristique Code Relay 3
  #define CHARACTERISTIC_UUID_292B         "0000292b-0000-1000-8000-00805f9b34fb"    // Caractéristique Terme de modification
  #define CHARACTERISTIC_UUID_292C         "0000292c-0000-1000-8000-00805f9b34fb"    // Caractéristique Signal Buzzer clavier 
  #define CHARACTERISTIC_UUID_292E         "0000292e-0000-1000-8000-00805f9b34fb"    // Caractéristique Number Digit 
  #define CHARACTERISTIC_UUID_292D         "0000292d-0000-1000-8000-00805f9b34fb"    // Caractéristique Master Code 
  #define CHARACTERISTIC_UUID_292A         "0000292a-0000-1000-8000-00805f9b34fb"    // Caractéristique Temporisations
  #define CHARACTERISTIC_UUID_5000         "00005000-0000-1000-8000-00805f9b34fb"    // Caractéristique 0
  #define CHARACTERISTIC_UUID_5001         "00005001-0000-1000-8000-00805f9b34fb"    // Caractéristique 1
  #define CHARACTERISTIC_UUID_5002         "00005002-0000-1000-8000-00805f9b34fb"    // Caractéristique 2
  #define CHARACTERISTIC_UUID_5003         "00005003-0000-1000-8000-00805f9b34fb"    // Caractéristique 3
  #define CHARACTERISTIC_UUID_5004         "00005004-0000-1000-8000-00805f9b34fb"    // Caractéristique 4
  #define CHARACTERISTIC_UUID_5005         "00005005-0000-1000-8000-00805f9b34fb"    // Caractéristique 5
  #define CHARACTERISTIC_UUID_5006         "00005006-0000-1000-8000-00805f9b34fb"    // Caractéristique 6
  #define CHARACTERISTIC_UUID_5007         "00005007-0000-1000-8000-00805f9b34fb"    // Caractéristique 7
  #define CHARACTERISTIC_UUID_5008         "00005008-0000-1000-8000-00805f9b34fb"    // Caractéristique 8
  #define CHARACTERISTIC_UUID_5009         "00005009-0000-1000-8000-00805f9b34fb"    // Caractéristique 9
  #define CHARACTERISTIC_UUID_5010         "00005010-0000-1000-8000-00805f9b34fb"    // Caractéristique 10
  #define CHARACTERISTIC_UUID_5011         "00005011-0000-1000-8000-00805f9b34fb"    // Caractéristique 11
  #define CHARACTERISTIC_UUID_5012         "00005012-0000-1000-8000-00805f9b34fb"    // Caractéristique 12
  #define CHARACTERISTIC_UUID_5013         "00005013-0000-1000-8000-00805f9b34fb"    // Caractéristique 13
  #define CHARACTERISTIC_UUID_5014         "00005014-0000-1000-8000-00805f9b34fb"    // Caractéristique 14
  #define CHARACTERISTIC_UUID_5015         "00005015-0000-1000-8000-00805f9b34fb"    // Caractéristique 15

//GALEO



class MyCallbacks_Door_Status: public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Door_Status)
    { std::string value = pK_Door_Status->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Door_Status*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Door_Status)
    { Serial.println("****pK_Door_Status*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Door_Status->setValue(txString);
      pK_Door_Status->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_Relay_State: public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Relay_State)
    { std::string value = pK_Relay_State->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Relay_State*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Relay_State)
    { Serial.println("****pK_Relay_State*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Relay_State->setValue(txString);
      pK_Relay_State->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};
////////////////////////////////////////////////////////////////////
class MyCallbacks_Crypto: public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Crypto)
    { std::string value = pK_Crypto->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Crypto*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Crypto)
    { Serial.println("****pK_Crypto*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Crypto->setValue(txString);
      pK_Crypto->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
} ;


class MyCallbacks_Crypto2: public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Crypto2)
    { std::string value = pK_Crypto2->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Crypto2*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Crypto2)
    { Serial.println("****pK_Crypto2*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Crypto2->setValue(txString);
      pK_Crypto2->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};
/////////////////////////////////////////////////////////////////////////

class MyCallbacks_Tempo : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Tempo)
    { std::string value = pK_Tempo->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Tempo*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Tempo)
    { Serial.println("****pK_Tempo*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Tempo->setValue(txString);
      pK_Tempo->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    };
    


class MyCallbacks_Term_Modif : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Term_Modif)
    { std::string value = pK_Term_Modif->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Term_Modif*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Term_Modif)
    { Serial.println("****pK_Term_Modif*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Term_Modif->setValue(txString);
      pK_Term_Modif->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_Buzzer_Key : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Buzzer_Key)
    { std::string value = pK_Buzzer_Key->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Buzzer_Key*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Buzzer_Key)
    { Serial.println("****pK_Buzzer_Key*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Buzzer_Key->setValue(txString);
      pK_Buzzer_Key->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_Master_Code : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Master_Code)
    { std::string value = pK_Master_Code->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Master_Code*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Master_Code)
    { Serial.println("****pK_Master_Code*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Master_Code->setValue(txString);
      pK_Master_Code->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_Num_Digit : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Num_Digit)
    { std::string value = pK_Num_Digit->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Num_Digit*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Num_Digit)
    { Serial.println("****pK_Num_Digit*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Num_Digit->setValue(txString);
      pK_Num_Digit->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};


class MyCallbacks_0 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_0)
    { std::string value = pK_0->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_0*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_0)
    { Serial.println("****pK_0*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_0->setValue(txString);
      pK_0->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_1 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_1)
    { std::string value = pK_1->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_1*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_1)
    { Serial.println("****pK_1*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_1->setValue(txString);
      pK_1->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};


class MyCallbacks_2 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_2)
    { std::string value = pK_2->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_2*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_2)
    { Serial.println("****pK_2*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_2->setValue(txString);
      pK_2->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_3 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_3)
    { std::string value = pK_3->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_3*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_3)
    { Serial.println("****pK_3*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_3->setValue(txString);
      pK_3->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_4 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_4)
    { std::string value = pK_4->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_4*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_4)
    { Serial.println("****pK_4*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_4->setValue(txString);
      pK_4->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_5 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_5)
    { std::string value = pK_5->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_5*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_5)
    { Serial.println("****pK_5*****");
      Serial.println("GALEO : demande de lecture ");
     // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_5->setValue(txString);
      pK_5->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_6 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_6)
    { std::string value = pK_6->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_6*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_6)
    { Serial.println("****pK_6*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_6->setValue(txString);
      pK_6->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_7 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_7)
    { std::string value = pK_7->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_7*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_7)
    { Serial.println("****pK_7*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_7->setValue(txString);
      pK_7->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_8 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_8)
    { std::string value = pK_8->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_8*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_8)
    { Serial.println("****pK_8*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_8->setValue(txString);
      pK_8->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_9 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_9)
    { std::string value = pK_9->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_9*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_9)
    { Serial.println("****pK_9*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_9->setValue(txString);
      pK_9->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_10 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_10)
    { std::string value = pK_10->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_10*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_10)
    { Serial.println("****pK_10*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_10->setValue(txString);
      pK_10->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_11 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_11)
    { std::string value = pK_11->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_11*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_11)
    { Serial.println("****pK_11*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_11->setValue(txString);
      pK_11->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_12 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_12)
    { std::string value = pK_12->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_12*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_12)
    { Serial.println("****pK_12*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_12->setValue(txString);
      pK_12->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_13 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_13)
    { std::string value = pK_13->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_13*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_13)
    { Serial.println("****pK_13*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_13->setValue(txString);
      pK_13->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_14 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_14)
    { std::string value = pK_14->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_14*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_14)
    { Serial.println("****pK_14*****");
      Serial.println("GALEO : demande de lecture ");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_14->setValue(txString);
      pK_14->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_15 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_15)
    { std::string value = pK_15->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_15*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_15)
    { Serial.println("****pK_15*****");
      Serial.println("GALEO : demande de lecture ");
        // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_15->setValue(txString);
      pK_15->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_Relay_1 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Relay_1)
    { std::string value = pK_Relay_1->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Relay_1*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Relay_1)
    { Serial.println("****pK_Relay_1*****");
      Serial.println("GALEO : demande de lecture ");
        // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Relay_1->setValue(txString);
      pK_Relay_1->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_Relay_1_Con : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Relay_1_Con)
    { std::string value = pK_Relay_1_Con->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Relay_1_Con*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Relay_1_Con)
    { Serial.println("****pK_Relay_1_Con*****");
      Serial.println("GALEO : demande de lecture ");
        // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Relay_1_Con->setValue(txString);
      pK_Relay_1_Con->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_Relay_2 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Relay_2)
    { std::string value = pK_Relay_2->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Relay_2*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Relay_2)
    { Serial.println("****pK_Relay_2*****");
      Serial.println("GALEO : demande de lecture ");
        // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Relay_2->setValue(txString);
      pK_Relay_2->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

class MyCallbacks_Relay_3 : public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Relay_3)
    { std::string value = pK_Relay_3->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Relay_3*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Relay_3)
    { Serial.println("****pK_Relay_3*****");
      Serial.println("GALEO : demande de lecture ");
       // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Relay_3->setValue(txString);
      pK_Relay_3->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};


///////////////////////////////////////////
class MyCallbacks_Manuf_Name: public BLECharacteristicCallbacks
{   
    void onWrite(BLECharacteristic *pK_Manuf_Name)
    { std::string value = pK_Manuf_Name->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Manuf_Name*****");
        Serial.print("GALEO : New value: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Manuf_Name)
    { Serial.println("****pK_Manuf_Name*****");
      Serial.println("GALEO : demande de lecture ");
       // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Manuf_Name->setValue(txString);
      pK_Manuf_Name->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }
    
};

//+++++++++++++++++++++++++++++++++
class MyCallbacks_Model_Number: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pK_Model_Number)
    { std::string value = pK_Model_Number->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Model_Number*****");
        Serial.print("Nouvelle valeur: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Model_Number)
    { Serial.println("****pK_Model_Number*****");
      Serial.println("demande de lecture 2");
        // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Model_Number->setValue(txString);
      pK_Model_Number->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }    
};

class MyCallbacks_Hardware_Rev: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pK_Hardware_Rev)
    { std::string value = pK_Hardware_Rev->getValue();
      if (value.length() > 0)
      { Serial.println("***pK_Hardware_Rev******");
        Serial.print("Nouvelle valeur: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Hardware_Rev)
    { Serial.println("***pK_Hardware_Rev******");
      Serial.println("demande de lecture 2");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Hardware_Rev->setValue(txString);
      pK_Hardware_Rev->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }    
};

class MyCallbacks_Firmware_Rev: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pK_Firmware_Rev)
    { std::string value = pK_Firmware_Rev->getValue();
      if (value.length() > 0)
      { Serial.println("***pK_Firmware_Rev******");
        Serial.print("Nouvelle valeur: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Firmware_Rev)
    { Serial.println("****pK_Firmware_Rev*****");
      Serial.println("demande de lecture 2");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Firmware_Rev->setValue(txString);
      pK_Firmware_Rev->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }    
};

class MyCallbacks_Soft_Rev: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pK_Soft_Rev)
    { std::string value = pK_Soft_Rev->getValue();
      if (value.length() > 0)
      { Serial.println("***pK_Soft_Rev******");
        Serial.print("Nouvelle valeur: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Soft_Rev)
    { Serial.println("****pK_Soft_Rev*****");
      Serial.println("demande de lecture 2");
      // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Soft_Rev->setValue(txString);
      pK_Soft_Rev->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }    
};

class MyCallbacks_Syst_Id: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pK_Syst_Id)
    { std::string value = pK_Syst_Id->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Syst_Id*****");
        Serial.print("Nouvelle valeur: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Syst_Id)
    { Serial.println("*****pK_Syst_Id****");
      Serial.println("demande de lecture 2");
                  // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Syst_Id->setValue(txString);
      pK_Syst_Id->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");
    }    
};

class MyCallbacks_Pnp_Id: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pK_Pnp_Id)
    { std::string value = pK_Pnp_Id->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Pnp_Id*****");
        Serial.print("Nouvelle valeur: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Pnp_Id)
    { Serial.println("****pK_Pnp_Id*****");
      Serial.println("demande de lecture 2");
            // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Pnp_Id->setValue(txString);
      pK_Pnp_Id->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");  
    }    
};

 class MyCallbacks_Seriel_Num: public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pK_Seriel_Num)
    { std::string value = pK_Seriel_Num->getValue();
      if (value.length() > 0)
      { Serial.println("****pK_Seriel_Num*****");
        Serial.print("Nouvelle valeur: ");
        for (int i = 0; i < value.length(); i++)
          Serial.print(value[i]);

        Serial.println();
        Serial.println("*********");
      }
    }

    void onRead(BLECharacteristic *pK_Seriel_Num)
    { Serial.println("****pK_Seriel_Num*****");
      Serial.println("demande de lecture 2");
            // Let's convert the value to a char array:

      char txString[3]={'1','.','0'}; // make sure this is big enuffz
      pK_Seriel_Num->setValue(txString);
      pK_Seriel_Num->notify(); // Send the value to the app!
      Serial.print("*** Sent Value: ");
      Serial.print(txString);
      Serial.println(" ***");  
    }    
};




//+++++++++++++++++++++++++++++++++

void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);

  Serial.println("1- Download and install an BLE scanner app in your phone");
  Serial.println("2- Scan for BLE devices in the app");
  Serial.println("3- Connect to MyESP32");
  Serial.println("4- Go to CUSTOM CHARACTERISTIC in CUSTOM SERVICE and write something");
  Serial.println("5- See the magic =)");

  

  BLEDevice::init("BX0988654517686C10");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService180A = pServer->createService(SERV_UUID_180A);
  BLEService *pService55AA = pServer->createService(SERV_UUID_55AA);
  BLEService *pService190B = pServer->createService(SERV_UUID_190B);
  BLEService *pService190BB = pServer->createService(SERV_UUID_190BB);
  BLEService *pService190A = pServer->createService(SERV_UUID_190A);



  
BLECharacteristic *pK_Crypto = pService55AA->createCharacteristic(
                                         CHARACTERISTIC_UUID_55AA,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Crypto->setCallbacks(new MyCallbacks_Crypto());
  pK_Crypto->setValue("pK_Crypto");

  BLECharacteristic *pK_Crypto2 = pService55AA->createCharacteristic(
                                         CHARACTERISTIC_UUID_55AAA,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Crypto2->setCallbacks(new MyCallbacks_Crypto2());
  pK_Crypto2->setValue("pK_Crypto2");

  
BLECharacteristic *pK_Door_Status = pService190A->createCharacteristic(
                                         CHARACTERISTIC_UUID_2931,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Door_Status->setCallbacks(new MyCallbacks_Door_Status());
  pK_Door_Status->setValue("pK_Door_Status");

BLECharacteristic *pK_Relay_State = pService190A->createCharacteristic(
                                         CHARACTERISTIC_UUID_292F,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Relay_State->setCallbacks(new MyCallbacks_Relay_State());
  pK_Relay_State->setValue("pK_Relay_State");

BLECharacteristic *pK_Tempo = pService190B->createCharacteristic(
                                         CHARACTERISTIC_UUID_292A,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Tempo->setCallbacks(new MyCallbacks_Tempo());
  pK_Tempo->setValue("pK_Tempo");
  
BLECharacteristic *pK_Term_Modif = pService190B->createCharacteristic(
                                         CHARACTERISTIC_UUID_292B,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Term_Modif->setCallbacks(new MyCallbacks_Term_Modif());
  pK_Term_Modif->setValue("pK_Term_Modif");

  BLECharacteristic *pK_Buzzer_Key = pService190B->createCharacteristic(
                                         CHARACTERISTIC_UUID_292C,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Buzzer_Key->setCallbacks(new MyCallbacks_Buzzer_Key());
  pK_Buzzer_Key->setValue("pK_Buzzer_Key");

  BLECharacteristic *pK_Master_Code = pService190B->createCharacteristic(
                                         CHARACTERISTIC_UUID_292D,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Master_Code->setCallbacks(new MyCallbacks_Master_Code());
  pK_Master_Code->setValue("pK_Master_Code");

  BLECharacteristic *pK_Num_Digit = pService190B->createCharacteristic(
                                         CHARACTERISTIC_UUID_292E,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Num_Digit->setCallbacks(new MyCallbacks_Num_Digit());
  pK_Num_Digit->setValue("pK_Num_Digit");

 BLECharacteristic *pK_0 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5000,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_0->setCallbacks(new MyCallbacks_0());
  pK_0->setValue("pK_0");


 BLECharacteristic *pK_1 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5001,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_1->setCallbacks(new MyCallbacks_1());
  pK_1->setValue("pK_1");


  BLECharacteristic *pK_2 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5002,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_2->setCallbacks(new MyCallbacks_2());
  pK_2->setValue("pK_2");

  BLECharacteristic *pK_3 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5003,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_3->setCallbacks(new MyCallbacks_3());
  pK_3->setValue("pK_3");

  BLECharacteristic *pK_4 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5004,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_4->setCallbacks(new MyCallbacks_4());
  pK_4->setValue("pK_4");

  BLECharacteristic *pK_5 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5005,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_5->setCallbacks(new MyCallbacks_5());
  pK_5->setValue("pK_5");

  BLECharacteristic *pK_6 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5006,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_6->setCallbacks(new MyCallbacks_6());
  pK_6->setValue("pK_6");

  BLECharacteristic *pK_7 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5007,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_7->setCallbacks(new MyCallbacks_7());
  pK_7->setValue("pK_7");

  BLECharacteristic *pK_8 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5008,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_8->setCallbacks(new MyCallbacks_8());
  pK_8->setValue("pK_8");

  BLECharacteristic *pK_9 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5009,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_9->setCallbacks(new MyCallbacks_9());
  pK_9->setValue("pK_9");

  BLECharacteristic *pK_10 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5010,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_10->setCallbacks(new MyCallbacks_10());
  pK_10->setValue("pK_10");

  BLECharacteristic *pK_11 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5011,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_11->setCallbacks(new MyCallbacks_11());
  pK_11->setValue("pK_11");

  BLECharacteristic *pK_12 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5012,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_12->setCallbacks(new MyCallbacks_12());
  pK_12->setValue("pK_12");

  BLECharacteristic *pK_13 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5013,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_13->setCallbacks(new MyCallbacks_13());
  pK_13->setValue("pK_13");

  BLECharacteristic *pK_14 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5014,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_14->setCallbacks(new MyCallbacks_14());
  pK_14->setValue("pK_14");

  BLECharacteristic *pK_15 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_5015,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_15->setCallbacks(new MyCallbacks_15());
  pK_15->setValue("pK_15");

BLECharacteristic *pK_Relay_1 = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_2932,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Relay_1->setCallbacks(new MyCallbacks_Relay_1());
  pK_Relay_1->setValue("pK_Relay_1");

BLECharacteristic *pK_Relay_1_Con = pService190BB->createCharacteristic(
                                         CHARACTERISTIC_UUID_2936,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Relay_1_Con->setCallbacks(new MyCallbacks_Relay_1_Con());
  pK_Relay_1_Con->setValue("pK_Relay_1_Con");

  BLECharacteristic *pK_Relay_2 = pService190B->createCharacteristic(
                                         CHARACTERISTIC_UUID_2933,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Relay_2->setCallbacks(new MyCallbacks_Relay_2());
  pK_Relay_2->setValue("pK_Relay_2");

  BLECharacteristic *pK_Relay_3 = pService190B->createCharacteristic(
                                         CHARACTERISTIC_UUID_2934,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pK_Relay_3->setCallbacks(new MyCallbacks_Relay_3());
  pK_Relay_3->setValue("pK_Relay_3");


BLECharacteristic *pK_Seriel_Num = pService180A->createCharacteristic(
                                         K_SERIAL_NUM_UUID_2A25 ,
                                         BLECharacteristic::PROPERTY_READ 
                                       );

  pK_Seriel_Num->setCallbacks(new MyCallbacks_Seriel_Num());
  pK_Seriel_Num->setValue("pK_Seriel_Num");

  BLECharacteristic *pK_Manuf_Name = pService180A->createCharacteristic(
                                         K_MANUF_NAME_UUID_2A29,
                                         BLECharacteristic::PROPERTY_READ 
                                       );

  pK_Manuf_Name->setCallbacks(new MyCallbacks_Manuf_Name());
  pK_Manuf_Name->setValue("pK_Manuf_Name");

  BLECharacteristic *pK_Model_Number = pService180A->createCharacteristic(
                                         K_MOD_NB_UUID_2A24,
                                         BLECharacteristic::PROPERTY_READ 
                                       );

  pK_Model_Number->setCallbacks(new MyCallbacks_Model_Number());
  pK_Model_Number->setValue("pK_Model_Number");

  BLECharacteristic *pK_Hardware_Rev = pService180A->createCharacteristic(
                                         K_HARD_REV_UUID_2A27,
                                         BLECharacteristic::PROPERTY_READ 
                                       );

  pK_Hardware_Rev->setCallbacks(new MyCallbacks_Hardware_Rev());
  pK_Hardware_Rev->setValue("pK_Hardware_Rev");

  BLECharacteristic *pK_Firmware_Rev = pService180A->createCharacteristic(
                                         K_FIRM_REV_UUID_2A26,
                                         BLECharacteristic::PROPERTY_READ 
                                       );

  pK_Firmware_Rev->setCallbacks(new MyCallbacks_Firmware_Rev());
  pK_Firmware_Rev->setValue("pK_Firmware_Rev");
  
  BLECharacteristic *pK_Soft_Rev = pService180A->createCharacteristic(
                                         K_SOFT_REV_UUID_2A28,
                                         BLECharacteristic::PROPERTY_READ 
                                       );

  pK_Soft_Rev->setCallbacks(new MyCallbacks_Soft_Rev());
  pK_Soft_Rev->setValue("pK_Soft_Rev");

  BLECharacteristic *pK_Syst_Id = pService180A->createCharacteristic(
                                         K_SYST_ID_UUID_2A23,
                                         BLECharacteristic::PROPERTY_READ 
                                       );

  pK_Syst_Id->setCallbacks(new MyCallbacks_Syst_Id());
  pK_Syst_Id->setValue("pK_Syst_Id");

  BLECharacteristic *pK_Pnp_Id = pService180A->createCharacteristic(
                                         K_PNP_ID_UUID_2A50,
                                         BLECharacteristic::PROPERTY_READ 
                                       );

  pK_Pnp_Id->setCallbacks(new MyCallbacks_Pnp_Id());
  pK_Pnp_Id->setValue("pK_Pnp_Id");


  //+++++++++++++++++++++++++++++++++++++++++++
/*  BLECharacteristic *pCharacteristic2 = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID2,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic2->setCallbacks(new MyCallbacks2());

  pCharacteristic2->setValue("Hello World2");*/
  //+++++++++++++++++++++++++++++++++++++++++++
  
  pService180A->start();
  pService190A->start();
  pService190B->start();
  pService55AA->start();
 


   pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
//  BLEAdvertising *pAdvertising = pServer->getAdvertising();
//  pAdvertising->start();
//  Serial.println("Waiting a client connection to notify...");
}

void loop() {
//   //put your main code here, to run repeatedly:
//
  if (deviceConnected) 
  {
    Serial.println("DEVICE CONNECTED");
//    // Fabricate some arbitrary junk for now...
//    txValue = analogRead(readPin) / 3.456; // This could be an actual sensor reading!
//    //TxValue = (uint16_t)(txValue*1);
//
//    // Let's convert the value to a char array:
//    char txString[16]; // make sure this is big enuffz
//    dtostrf(txValue, 1, 2, txString); // float_val, min_width, digits_after_decimal, char_buffer
//    
//   // pCharacteristic->setValue(&txValue,1); // To send the integer value
//    pK_Relay_2->setValue("Hello!"); // Sending a test message
//    pK_Relay_1->setValue(txString);
//    
//    pK_Master_Code ->notify(); // Send the value to the app!
//    Serial.print("*** Sent Value: ");
//    Serial.print(txString);
//    Serial.println(" ***");
  }
delay(2000);
}

