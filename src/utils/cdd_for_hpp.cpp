#include <hpp/walkingctrl/utils/cdd_for_hpp.hh>
#include <ctime>

namespace hpp{
    namespace walkingctrl{
        void init_cdd_library(){
            dd_set_global_constants();
            dd_debug = false;
        }
        void release_cdd_library(){

        }
        dd_MatrixPtr cone_span_eigen_to_cdd(CRef_matrix_t input, const bool canonicalize)        {
            dd_debug = false;
            dd_MatrixPtr M=NULL;
            dd_rowrange i;
            dd_colrange j;
            dd_rowrange m_input = (dd_rowrange)(input.rows());
            dd_colrange d_input = (dd_colrange)(input.cols()+1);
            dd_RepresentationType rep=dd_Generator;
            mytype value;
            dd_NumberType NT = dd_Real;
            dd_init(value);

            M=dd_CreateMatrix(m_input, d_input);
            M->representation=rep;
            M->numbtype=NT;

            for (i = 0; i < input.rows(); i++)
            {
                dd_set_d(value, 1);
                dd_set(M->matrix[i][0],value);
                for (j = 1; j < d_input; j++)
                {
                    dd_set_d(value, input(i,j-1));
                    dd_set(M->matrix[i][j],value);
                }
            }
            dd_clear(value);
            if(canonicalize)
            {
                dd_ErrorType error = dd_NoError;
                dd_rowset redset,impl_linset;
                dd_rowindex newpos;
                dd_MatrixCanonicalize(&M, &impl_linset, &redset, &newpos, &error);
                set_free(redset);
                set_free(impl_linset);
                free(newpos);
            }

            return M;
        }

        void compute_convex_hull(CRef_matrix_t input, matrix_t& B_sp, vector_t& b_sp){
            init_cdd_library();

            dd_MatrixPtr V=cone_span_eigen_to_cdd(input, false);

            dd_ErrorType error = dd_NoError;
            dd_PolyhedraPtr H = dd_DDMatrix2Poly(V, &error);
            dd_MatrixPtr b_A = dd_CopyInequalities(H);

            int rowsize = (int)b_A->rowsize;

            B_sp.resize(rowsize, (int)b_A->colsize-1);
            b_sp.resize(rowsize);
            for(int i=0; i < rowsize; ++i)
            {
                b_sp(i) = (value_type)(*(b_A->matrix[i][0]));
                for(int j=1; j < b_A->colsize; ++j)
                B_sp(i, j-1) = (value_type)(*(b_A->matrix[i][j]));
            }   

        }

    }
}
