// 
// . the start/end vertex index
// {3, 10}
// 
#ifndef _COARSESUF_CURVE_INFO_H_
#define _COARSESUF_CURVE_INFO_H_

namespace coarseSuf
{

class CurveInfo {
public:
	CurveInfo();
	~CurveInfo();

	int start;
	int end;
	int len;
private:
};
    
}

#endif
