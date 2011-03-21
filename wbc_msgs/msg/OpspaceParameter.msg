int64 PARAMETER_TYPE_VOID    = 0
int64 PARAMETER_TYPE_STRING  = 1
int64 PARAMETER_TYPE_INTEGER = 2
int64 PARAMETER_TYPE_REAL    = 3
int64 PARAMETER_TYPE_VECTOR  = 4
int64 PARAMETER_TYPE_MATRIX  = 5

string name
int64 type

##################################################
# only for PARAMETER_TYPE_STRING

string strval

##################################################
# only for PARAMETER_TYPE_INTEGER

int64 intval

##################################################
# for PARAMETER_TYPE_REAL, realval will be of length 1 (one) and
# contain "the" value
#
# for PARAMETER_TYPE_VECTOR, realval is "the" value
#
# for PARAMETER_TYPE_MATRIX, realval is the value as mapped to
# contiguous memory by Eigen::Map<...>, and the dimensions of the
# matrix are stored in nrows, ncols

float64[] realval

# only for PARAMETER_TYPE_MATRIX

int64 nrows

# only for PARAMETER_TYPE_MATRIX

int64 ncols
