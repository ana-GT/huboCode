class ZMPGenerator
{
private:
	int m_nSPointNum;
	int m_nDPointNum;
	VectorXd m_vZMP;

public:
	ZMPGenerator(int nSPointNum, int nDPointNum);
	ZMPGen(double dStart, double dEnd);


};

ZMPGenerator::ZMPGenerator(int nSPointNum, int nDPointNum)
	:m_nSPointNum(nSPointNum), m_nDPointNum(nDPointNum)
{
	m_vZMP.resize(nDPointNum + nSPointNum);
}

ZMPGenerator::ZMPGen(double dStart, double dEnd)
{
	double dx1, dx2, dy1, dy2;
	double da, db;

	dx1 = 1;
	dy1 = dStart;

	dx2 = DPointNum;
	dy2 = dEnd;

	da = dStart, dEnd;
	db = dEnd, dStart;

	
	x = 1:DPointNum;
	t = (x-x1)/(x2-x1);

	q = (1-t)*y1 + t*y2 + t.*(1-t).*(a*(1-t)+b*t);

	for i = DPointNum+1:DPointNum + SPointNum
	   q(i) = endval;

	end

