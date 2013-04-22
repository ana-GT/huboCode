


enum Stance
{
	STANCE_SINGLE_LEFT,
	STANCE_SINGLE_RIGHT,
	STANCE_DUAL_LEFT,
	STANCE_DUAL_RIGHT
};


enum Foot
{
	FOOT_LEFT;
	FOOT_RIGHT;
};


class ZMPWalker
{
private:
	ZMPPatternGenerator m_cZMPPatternGenerator;		// for generating ZMP pattern based on footsteps
	PreviewController m_cPreviewController;			// for preview controlling
	COMIKApplier m_cCOMIKApplier;					// for generating joint commands based on COM info

	int nSingleSupport;
	int nDoubleSupport;


public:
	ZMPWalker(const ZMPPatternGenerator &cZMPPatternGenerator, const PreviewController &cPC, const COMIKApplier &CIK)
		: m_cZMPPatternGenerator(cZMPPatternGenerator) m_cPreviewController(cPC), m_cCOMIKApplier(CIK)
	{}
	void ZMPWalk();
};

void ZMPWalker::ZMPWalk()
{
	std::queue<FootStep> FootStepQueue;
	/***************************
	 * Initialization work
	 **************************/
	Init();

	/***********************************
	 * Continuously read new foot steps
	 ***********************************/
	while (true) {
		FootStep cNewFootStep = GetNewFootStep();
		m_cZMPPatternGenerator.ZMPPatternGenerate(cNewFootStep);
	}


}


class ZMPPatternGenerator()
{
public:
	ZMPPatternGenerator


/********************************
 * Generate foot steps one by one
 ********************************/
class FootStepGeneraor
{
public:
	FootStep GetNewFootStep();

};


/***************************
 * Results of one foot step
 ***************************/
class FootStep
{
public:
	FootStep(Foot eFoot, FootPosture cLeftFootPosture, FootPosture cRightFootPosture) 
		: m_eFoot(eFoot), m_cLeftFootPosture(cLeftFootPosture), m_cRightFootPosture(cRightFootPosture)
	{}

private:
	Foot m_eFoot;
	FootPosture m_cLeftFootPosture;
	FootPosture m_cRightFootPosture;
};


/*******************************
 * Foot Posture for one foot
 ******************************/
class FootPosture
{
public:
	FootPosture(double dX, double dY, double dYaw)
		: m_dX(dX), m_dY(dY), m_dYaw(dYaw)
	{}

private:
	double m_dX; 
	double m_dY; 
	double m_dYaw;
};

class ZMPPoint
{
private:
	double m_dX;
	double m_dY;
};
