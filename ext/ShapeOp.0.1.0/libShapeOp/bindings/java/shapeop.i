/* Following http://www.swig.org/tutorial.html */
%module shapeopJava

 /* Adding c-array proxies (http://www.swig.org/Doc3.0/SWIGDocumentation.html#Library_carrays) */
 %include "carrays.i"
 %array_class(int, intArray);
 %array_class(double, doubleArray);

 %{
 #include "Common.h"
 #include "API.h"
 %}

 %include "Common.h"
 %include "API.h"
