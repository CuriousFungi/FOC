  
#ifndef LIMIT_HPP
#define LIMIT_HPP

//=============================================================================
//                               Limit
//=============================================================================
template <typename VAR_TYPE>
class Limit
{
public:
  
    explicit Limit( VAR_TYPE min_val,
                    VAR_TYPE max_val)
             :   MIN_VAL(min_val)
             ,   MAX_VAL(max_val)
             {
                return;
             }

            ~Limit(){};           

    //------------------------------------------------------------------------- 
    //                            result
    //------------------------------------------------------------------------- 
    VAR_TYPE result(VAR_TYPE input)
    {
        VAR_TYPE limited_val = input;

        if(input > MAX_VAL) 
        {
            limited_val = MAX_VAL;
        }

        if(input < MIN_VAL)
        {
            limited_val = MIN_VAL;
        }

        return limited_val;
    } 


private:
  
    const VAR_TYPE      MIN_VAL;    
    const VAR_TYPE      MAX_VAL;     
};

#endif // inclusion guard
