#include "PreviewController.h"


int
main()
{

	PreviewController myPreviewController(0.005, 0.85, 1e7, 10.0);
	myPreviewController.PrintSaveGains();

}