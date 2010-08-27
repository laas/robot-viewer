Server IDL
**********

.. toctree::
   :maxdepth: 3

.. code-block:: c++

   #ifndef ROBOTVIEWER_IDL
   #define ROBOTVIEWER_IDL

   module hpp {
     typedef sequence<double> DoubleSeq;
     typedef sequence<string> ElementList;

     interface RobotViewer
     {
       // Write functions
       exception InvalidKey{
         string reason;
       };

       void createElement(in string type, in string name, in string description);

       void destroyElement(in string name) raises(InvalidKey);
       void enableElement(in string name) raises(InvalidKey) ;
       void disableElement(in string name) raises(InvalidKey) ;
       void updateElementConfig(in string name, in DoubleSeq config) raises(InvalidKey);
       void getElementConfig(in string name, out DoubleSeq config) raises(InvalidKey);

       void listElement(out ElementList list);
       void Ping(out string outstring);
     };
   };
   #endif
