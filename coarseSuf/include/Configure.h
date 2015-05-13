#ifndef _COARSESUF_CONFIGURE_H_
#define _COARSESUF_CONFIGURE_H_

#ifndef COARSESUF_OUTPUT
#define COARSESUF_OUTPUT 0
#endif

namespace coarseSuf
{
    
#define numResvOneTile 3300000
#define numResvTile 2400000
#define numResvHash 3300000
#define numResvTilling 200
#define MAXK 6

#define _SaveTetTri	0 // save tet triangles from tetgen
#define _SaveTile 1

#define hasTri false
#define doPause false
#define plainPTB 0.00000001
#define BADEDGE_LIMIT 30
#define hfPI 1.570796

#if _WIN32
    typedef __int64 int64;
#else
    typedef long long int64;
#endif
    
}

#endif
