Server IDL
**********

.. toctree::
   :maxdepth: 3

.. code-block:: c++

    #ifndef ROBOTVIEWER_IDL
    #define ROBOTVIEWER_IDL

    module robotviewer_corba {
      typedef sequence<double> DoubleSeq;
      typedef sequence<string> ElementList;

      interface RobotViewer
      {
	exception InvalidKey{
	  string reason;
	};

	boolean createElement(in string type, in string name, in string description);
	boolean setRobotJointRank(in string name, in string path);
	boolean destroyElement(in string name) raises(InvalidKey);
	boolean enableElement(in string name) raises(InvalidKey) ;
	boolean disableElement(in string name) raises(InvalidKey) ;
	boolean updateElementConfig(in string name, in DoubleSeq config) raises(InvalidKey);
	void getElementConfig(in string name, out DoubleSeq config) raises(InvalidKey);

	void listElements(out ElementList list);
	void listElementDofs(in string element_name, out ElementList list);
	void Ping(out string outstring);
      };
    };
    #endif
