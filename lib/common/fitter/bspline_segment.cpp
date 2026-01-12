#include "bspline_segment.h"

namespace cem {
namespace fusion {

int ts_fequals(float x, float y)
{
    if (fabs(x-y) <= 1e-5) 
    {
        return 1;
    }
    else 
    {
        const float r = (float)fabs(x) > (float)fabs(y) ? (float)fabs((x-y) / x) : (float)fabs((x-y) / y);
        return r <= 1e-8;
    }
}
//curve fitting optimization.
int judgePointInPnPoly(const std::vector<Point2DF> &vertices, const float &x, const float &y)
{
    int result = 0;
    float xMin = FLT_MAX;
    float yMin = FLT_MAX;
    float xMax = -FLT_MAX;
    float yMax = -FLT_MAX;

    for(size_t idx = 0; idx < vertices.size(); idx++)
    {
        if(vertices[idx].x < xMin)
        {
            xMin = vertices[idx].x;
        }
        if(vertices[idx].y < yMin)
        {
            yMin = vertices[idx].y;
        }
        if(vertices[idx].x > xMax)
        {
            xMax = vertices[idx].x;
        }
        if(vertices[idx].y > yMax)
        {
            yMax = vertices[idx].y;
        }
    }

    if(x < xMin || x > xMax || y < yMin || y > yMax)
    {
        return result;
    }

    size_t i, j;
    for (i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++) 
    {
        if(((vertices[i].y > y) != (vertices[j].y > y)) &&
           (x < (((vertices[j].x - vertices[i].x) * (y - vertices[i].y)) / (vertices[j].y - vertices[i].y ) + vertices[i].x)))
        {
            result = !result;
        }
    }

    return result;
}
void computeRectangleWithVector(
    const Eigen::Vector2f & first,
    const Eigen::Vector2f & second,
    const float & width,
    const float & length,
    std::vector<Point2DF> &vertices)
{
    float deltaX = second(0) - first(0);
    float deltaY = second(1) - first(1);
    
    if((fabs(deltaX) < 1e-5 && fabs(deltaY) < 1e-5) || 
       (fabs(width) < 1e-5 || fabs(length) < 1e-5))
    {
        return;
    }
    Eigen::Vector2f majorVec(deltaX, deltaY);
    Eigen::Vector2f majorUnitVec = majorVec / majorVec.norm();
    Eigen::Vector2f newReversePoint;
    
    Eigen::Vector2f positionCenter;
    positionCenter(0) = first(0) + (length/2.0)*majorUnitVec(0);
    positionCenter(1) = first(1) + (length/2.0)*majorUnitVec(1);

    newReversePoint(0) = positionCenter(0) + 1.0 * majorUnitVec(0);
    newReversePoint(1) = positionCenter(1) + 1.0 * majorUnitVec(1);

    Eigen::Vector2f reverseMajorVec;
    reverseMajorVec(0) = positionCenter(0) - 1.0 * newReversePoint(0);
    reverseMajorVec(1) = positionCenter(1) - 1.0 * newReversePoint(1);

    Eigen::Vector2f reverseMajorUnitVec = reverseMajorVec / reverseMajorVec.norm();

    float theta = atan2(width, length);
    float newCosLength = (length/2.0)*cos(theta);
    float extendLength = (width/2.0)*sin(theta);

    Eigen::Vector2f majorPts, reverseMajorPts;
    majorPts(0) = newCosLength * majorUnitVec(0);
    majorPts(1) = newCosLength * majorUnitVec(1);

    reverseMajorPts(0) = newCosLength * reverseMajorUnitVec(0);
    reverseMajorPts(1) = newCosLength * reverseMajorUnitVec(1);

    Point2DF A, B;
    A.x = positionCenter(0) + majorPts(0) * cos(-theta) + majorPts(1) * sin(-theta);
    A.y = positionCenter(1) - majorPts(0) * sin(-theta) + majorPts(1) * cos(-theta);

    B.x = positionCenter(0) + majorPts(0) * cos(theta) + majorPts(1) * sin(theta);
    B.y = positionCenter(1) - majorPts(0) * sin(theta) + majorPts(1) * cos(theta);

    Eigen::Vector2f extendAUnitVec,extendBUnitVec;
    extendAUnitVec(0) = majorUnitVec(0) * cos(-theta) + majorUnitVec(1) * sin(-theta);
    extendAUnitVec(1) = -majorUnitVec(0) * sin(-theta) + majorUnitVec(1) * cos(-theta);

    extendBUnitVec(0) = majorUnitVec(0) * cos(theta) + majorUnitVec(1) * sin(theta);
    extendBUnitVec(1) = -majorUnitVec(0) * sin(theta) + majorUnitVec(1) * cos(theta);

    A.x = A.x + extendLength*extendAUnitVec(0);
    A.y = A.y + extendLength*extendAUnitVec(1);

    B.x = B.x + extendLength*extendBUnitVec(0);
    B.y = B.y + extendLength*extendBUnitVec(1);

    vertices.push_back(A);
    vertices.push_back(B);

    Point2DF C, D;
    C.x = positionCenter(0) + reverseMajorPts(0) * cos(-theta) + reverseMajorPts(1) * sin(-theta);
    C.y = positionCenter(1) - reverseMajorPts(0) * sin(-theta) + reverseMajorPts(1) * cos(-theta);

    D.x = positionCenter(0) + reverseMajorPts(0) * cos(theta) + reverseMajorPts(1) * sin(theta);
    D.y = positionCenter(1) - reverseMajorPts(0) * sin(theta) + reverseMajorPts(1) * cos(theta);

    Eigen::Vector2f extendCUnitVec;
    Eigen::Vector2f extendDUnitVec;
    extendCUnitVec(0) = reverseMajorUnitVec(0) * cos(-theta) + reverseMajorUnitVec(1) * sin(-theta);
    extendCUnitVec(1) = -reverseMajorUnitVec(0) * sin(-theta) + reverseMajorUnitVec(1) * cos(-theta);

    extendDUnitVec(0) = reverseMajorUnitVec(0) * cos(theta) + reverseMajorUnitVec(1) * sin(theta);
    extendDUnitVec(1) = -reverseMajorUnitVec(0) * sin(theta) + reverseMajorUnitVec(1) * cos(theta);

    C.x = C.x + extendLength*extendCUnitVec(0);
    C.y = C.y + extendLength*extendCUnitVec(1);

    D.x = D.x + extendLength*extendDUnitVec(0);
    D.y = D.y + extendLength*extendDUnitVec(1);


    vertices.push_back(C);
    vertices.push_back(D);

    return;
}
bool FitByBSpline(
    const std::vector<Eigen::Vector2f> & raw_points, 
    const int &cubic,
    std::vector<Eigen::Vector2f> &fitted_point_list)
{
    bool ret_val = true;
    size_t n_ctrlp = raw_points.size();
    size_t dim = 2;
    size_t degree = static_cast<size_t>(cubic);
    size_t deg = degree;
    size_t order = degree + 1;
    size_t num_knots = n_ctrlp + order;
    size_t len_ctrlp = n_ctrlp * dim;

    if (dim < 1 || num_knots > 10000 || 
        degree >= n_ctrlp || n_ctrlp <= 0 ||
       (num_knots - 2*deg - 1) == 0)
    {
        AINFO << "FitByBSpline() is out of range !";
        return !ret_val;
    }

    //compute knots and fill knots.
    std::vector<float> knots; /* the knots of \p _result_. */
    knots.resize(num_knots,0.0);

    /* n_knots >= 2*order == 2*(deg+1) == 2*deg + 2 > 2*deg - 1 */
    float fac = (1.0f - 0.0f) / (num_knots - 2*deg - 1);
    size_t knot_idx = order;
    for(;knot_idx < (num_knots-order); knot_idx++)
    {
        knots[knot_idx] = (knot_idx-deg)*fac;
    }
    for(size_t j = knot_idx; j < num_knots; j++)
    {
        knots[j] = 1.0;
    }

    //set control points data
    std::vector<float> ctrlp;
    ctrlp.resize(len_ctrlp + num_knots,0.0);
    size_t index = 0;
    for(size_t i = 0; i < n_ctrlp; i++)
    {
        ctrlp[index++] = raw_points[i](0);
        ctrlp[index++] = raw_points[i](1);
    }

    //evaluate
    //ts_int_deboornet_new()
    size_t nums = n_ctrlp;
    float u = 0.0; //knot
    size_t num_points = (size_t)(order * (order+1) * 0.5f);
    /* Handle `order == 1' which generates too few points. */
    size_t fixed_num_points = num_points < 2 ? 2 : num_points;
    float prev_x = 0.0,prev_y = 0.0;
    for(int idx = 0; idx < nums; idx++)
    {
        size_t n_points = fixed_num_points; /** Number of points in `points'. */
        std::vector<float> points;
        points.resize(fixed_num_points*dim);

        size_t k;        /**< Index of \p u. */
        size_t s;        /**< Multiplicity of \p u. */

        size_t from;     /**< Offset used to copy values. */
        size_t fst;      /**< First affected control point, inclusive. */
        size_t lst;      /**< Last affected control point, inclusive. */
        size_t N;        /**< Number of affected control points. */

        /* The following indices are used to create the DeBoor net. */
        size_t lidx;     /**< Current left index. */
        size_t ridx;     /**< Current right index. */
        size_t tidx;     /**< Current to index. */
        size_t r,i, d;  /**< Used in for loop. */
        float ui;       /**< Knot value at index i. */
        float a, a_hat; /**< Weighting factors of control points. */

        /* 1. Find index k such that u is in between [u_k, u_k+1).
         * 2. Setup already known values.
         * 3. Decide by multiplicity of u how to calculate point P(u). 
         */

        //ts_internal_bspline_find_u()
        k = s = 0;
        for(; k < num_knots; k++) 
        {
            float uk = knots[k];
            if(ts_fequals(u,uk))  
            {
                s++;
            }
            else if (u < uk) 
            {
                break;
            }
        }
        
        /* keep in mind that currently k is k+1 */
        if ((k <= deg)/* u < u_min */ || 
            (k == num_knots && s == 0)/* u > u_last */ || 
            (k > (num_knots-deg + s-1)) /* u > u_max */)      
        {
            AINFO << "ts_internal_bspline_find_u() is out of range !";
            return !ret_val;
        }          

        k--; /* k+1 - 1 will never underflow */
        float uk = knots[k];  /* Ensures that with any precision of  */
        if(ts_fequals(u,uk)) 
        {
            u = uk;
        }

        size_t sof_ctrlp = dim; 
        /* 2. */
        size_t h = deg < s ? 0 : deg-s; /* prevent underflow */

        /* 3. (by 1. s <= order)
         *
         * 3a) Check for s = order.
         *     Take the two points k-s and k-s + 1. If one of
         *     them doesn't exist, take only the other.
         * 3b) Use de boor algorithm to find point P(u). */

        if (s == order) 
        {
            /* only one of the two control points exists */
            if (k == deg || /* only the first */k == num_knots - 1) 
            { /* only the last */
                from = k == deg ? 0 : (k-s) * dim;
                n_points = 1;
                for(size_t n = 0; n < sof_ctrlp; n++)
                {
                    if((from + n) >= 0 && (from + n) < ctrlp.size())
                    {
                        points[n]  = ctrlp[from + n];
                    }
                    else
                    {
                        AINFO << "1 (from + n) is out of range !";
                        return !ret_val;
                    }
                }
            } 
            else 
            {
                from = (k-s) * dim;
                n_points = 2;
                for(size_t n = 0; n < 2*sof_ctrlp; n++)
                {
                    if((from + n) >= 0 && (from + n) < ctrlp.size())
                    {
                        points[n]  = ctrlp[from + n];
                    }
                    else
                    {
                        AINFO << "2 (from + n) is out of range !";
                        return !ret_val;
                    }
                }
            }
        } 
        else 
        { /* by 3a) s <= deg (order = deg+1) */
            fst = k-deg; /* by 1. k >= deg */
            lst = k-s; /* s <= deg <= k */
            N = lst-fst + 1; /* lst <= fst implies N >= 1 */

            n_points = (size_t)(N * (N+1) * 0.5f);
            /* copy initial values to output */
            for(size_t n = 0; n < N*sof_ctrlp; n++)
            {
                if((fst*dim + n) >= 0 && (fst*dim + n) < ctrlp.size())
                {
                    points[n]  = ctrlp[fst*dim + n];
                }
                else
                {
                    AINFO << "(fst*dim + n) is out of range !";
                    return !ret_val;
                }
            }

            lidx = 0;
            ridx = dim;
            tidx = N*dim; /* N >= 1 implies tidx > 0 */
            r = 1;
            for (;r <= h; r++) 
            {
                i = fst + r;
                for (; i <= lst; i++) 
                {
                    ui = knots[i];
                    if(fabs((knots[i+deg-r+1] - ui)) < 1e-5)
                    {
                        AINFO << "(fabs((knots[i+deg-r+1] - ui)) < 1e-5) !";
                        return !ret_val;
                    }
                    a = (u - ui) / (knots[i+deg-r+1] - ui);
                    a_hat = 1.f-a;

                    for (d = 0; d < dim; d++)
                    {
                        points[tidx++] = a_hat * points[lidx++] + a * points[ridx++];
                    }
                }
                lidx += dim;
                ridx += dim;
            }
        }

        if(points.size() > 0)
        {
            int type = (n_points == 2) ? 2 : 1;
            if(type == 2)
            {
                Eigen::Vector2f current_point;
                current_point(0) = static_cast<float>(points[0]);
                current_point(1) = static_cast<float>(points[1]);
                float delta_x = current_point(0) - prev_x;
                float delta_y = current_point(1) - prev_y;
                if(idx == 0)
                {
                    fitted_point_list.emplace_back(current_point);
                }
                else if(!(fabs(delta_x) < 1E-5 && fabs(delta_y) < 1E-5))
                {
                    fitted_point_list.emplace_back(current_point);
                }
                prev_x = current_point(0);
                prev_y = current_point(1);
            }
            else if(n_points <= points.size())
            {
                /* Last point in `points`. */
                size_t result_size =  n_points*dim - dim;
                Eigen::Vector2f current_point;
                current_point(0) = static_cast<float>(points[result_size]);
                current_point(1) = static_cast<float>(points[result_size+1]);
                float delta_x = current_point(0) - prev_x;
                float delta_y = current_point(1) - prev_y;
                if(idx == 0)
                {
                    fitted_point_list.emplace_back(current_point);
                }
                else if(!(fabs(delta_x) < 1E-5 && fabs(delta_y) < 1E-5))
                {
                    fitted_point_list.emplace_back(current_point);
                }
                prev_x = current_point(0);
                prev_y = current_point(1);
            }
            else
            {
                AINFO << "FALIURE: n_points > points.size()!";
                return !ret_val;
            }
        }
        else
        {
            AINFO << "FALIURE: points.size() zero!";
            return !ret_val;
        }

        if(nums > 1)
        {
            u += (1.0/(nums - 1));
        }
        else
        {
            u += (1.0/nums);
        }
    }

    return ret_val;
}

void GetSegsByBSplineAndRectangle(
    const std::vector<Point2DF> & seg, 
    std::vector<std::vector<Point2DF>>& seg_groups)
{
    // AINFO << "GetSegsByBSplineAndRectangle enter: seg.size() :"<<seg.size();
    if(seg.size() > 0)
    {
        const float RECT_WIDTH = 3.0;
        const float RECT_LENGTH = 210.0;
        const int MIN_POINTS = 20;
        const int SEG_LINE_MIN_POINTS = 10;
        const float FIRST_FRONT_SEGMENT_LENGTH = 30.0;

        std::vector<Eigen::Vector2f> raw_points;
        for(size_t point_index = 0 ; point_index < seg.size(); point_index++)
        {
            Eigen::Vector2f point;
            point(0) = seg[point_index].x;
            point(1) = seg[point_index].y;
            raw_points.push_back(point);
        }

        if(raw_points.size() >= 5)
        {
            std::vector<Eigen::Vector2f> fitted_points;
            // AINFO << "FitByBSpline() enter";
            if(true != FitByBSpline(raw_points,3,fitted_points))
            {
                std::vector<Eigen::Vector2f>().swap(fitted_points);
                fitted_points.assign(raw_points.begin(),raw_points.end());
                // AINFO << "FitByBSpline() failed";
            }

            std::vector<std::vector<size_t> > forward_segmented_indice;
            std::vector<std::vector<size_t> > backward_segmented_indice;
            std::vector<size_t> forward_part_indice;
            std::vector<size_t> backward_part_indice;

            int fitted_size = static_cast<int>(fitted_points.size());
            //searching
            if(fitted_size  > MIN_POINTS)
            {
                int positiveFirstIndex = -1;
                bool front_exist = false;
                for(size_t pointIndex = 0; pointIndex < fitted_points.size(); pointIndex++)
                {
                    if(fitted_points[pointIndex](0) >= 0.0)
                    {
                        positiveFirstIndex = static_cast<int>(pointIndex);
                        front_exist = true;
                        break;
                    }
                }
                if(true == front_exist && positiveFirstIndex >= static_cast<int>((fitted_points.size() - 2)))
                {
                    positiveFirstIndex = 0;
                }
                
                int workIndex = positiveFirstIndex;
                int segmentID = 0;
                while(workIndex < fitted_size && true == front_exist && positiveFirstIndex >= 0)
                {
                    Eigen::Vector2f first;
                    Eigen::Vector2f second;
                    first(0) = fitted_points[workIndex](0);
                    first(1) = fitted_points[workIndex](1);
                    if(workIndex < (fitted_size - 2))
                    {
                        second(0) = fitted_points[workIndex+2](0);
                        second(1) = fitted_points[workIndex+2](1);
                    }
                    else
                    {
                        second(0) = fitted_points[workIndex+1](0);
                        second(1) = fitted_points[workIndex+1](1);
                    }

                    std::vector<size_t> workdIndice;
                    workdIndice.push_back(workIndex);
                    if(0 == segmentID)
                    {
                        forward_part_indice.push_back(workIndex);
                    }
                    std::vector<Point2DF> vertices;
                    computeRectangleWithVector(first,second,RECT_WIDTH,RECT_LENGTH,vertices);
                    size_t fwdLastIndex = 0;
                    for(size_t positiveIndex = workIndex+1; positiveIndex < fitted_points.size(); positiveIndex++)
                    {
                        if(true == judgePointInPnPoly(vertices,fitted_points[positiveIndex](0),fitted_points[positiveIndex](1)))
                        {
                            if(0 == segmentID)
                            {
                                forward_part_indice.push_back(positiveIndex);
                            }
                            workdIndice.push_back(positiveIndex);
                            if(positiveIndex > fwdLastIndex)
                            {
                                fwdLastIndex = positiveIndex;
                            }
                        }
                        else
                        {
                            break;
                        }
                    }
                    /*Support DNP's PnC  using coeff C2, the length of the first segment in front of car must 50 meters.*/
                    if(0 == segmentID)
                    {
                        for(size_t tmpIdx = positiveFirstIndex; tmpIdx < fitted_points.size(); tmpIdx++)
                        {
                            if(tmpIdx < fitted_points.size())
                            {
                                float x = fitted_points[tmpIdx](0);
                                if(tmpIdx > fwdLastIndex && x < FIRST_FRONT_SEGMENT_LENGTH)
                                {
                                    forward_part_indice.push_back(tmpIdx);
                                    workdIndice.push_back(tmpIdx);
                                    fwdLastIndex = tmpIdx;
                                }
                                if(x >= FIRST_FRONT_SEGMENT_LENGTH)
                                {
                                    break;
                                }
                            }
                        }
                    }

                    if(fwdLastIndex > 0 && fwdLastIndex < fitted_points.size())
                    {
                        if(fwdLastIndex >= (fitted_points.size() - SEG_LINE_MIN_POINTS))
                        {
                            for(size_t positiveIndex = fwdLastIndex+1; positiveIndex < fitted_points.size(); positiveIndex++)
                            {
                                workdIndice.push_back(positiveIndex);
                                if(0 == segmentID)
                                {
                                    forward_part_indice.push_back(positiveIndex);
                                }
                            }
                            workIndex = fitted_size;  
                        }
                        else
                        {
                            workIndex = fwdLastIndex + 1; 
                        }
                        if(segmentID > 0)
                        {
                            forward_segmented_indice.push_back(workdIndice);  
                        }

                        //next segment
                        segmentID++;
                        std::vector<size_t>().swap(workdIndice); 
                    }
                    else
                    {
                        forward_segmented_indice.push_back(workdIndice);  
                        //next segment
                        segmentID++;
                        workIndex++;
                        std::vector<size_t>().swap(workdIndice); 
                    }
                }

                int invWorkIndex = positiveFirstIndex-1;
                if(false == front_exist)
                {//all is rear
                    invWorkIndex = fitted_points.size() - 1;
                }
                int invSegmentID = 0;
                std::vector<size_t> invWorkdIndice;
                while(invWorkIndex >= 1)
                {
                    Eigen::Vector2f invFirst;
                    Eigen::Vector2f invSecond;
                    invFirst(0) = fitted_points[invWorkIndex](0);
                    invFirst(1) = fitted_points[invWorkIndex](1);
                    invSecond(0) = fitted_points[invWorkIndex-1](0);
                    invSecond(1) = fitted_points[invWorkIndex-1](1);

                    invWorkdIndice.push_back(invWorkIndex);
                    if(0 == invSegmentID)
                    {
                        backward_part_indice.push_back(invWorkIndex);
                    }

                    std::vector<Point2DF> vertices;
                    computeRectangleWithVector(invFirst,invSecond,RECT_WIDTH,RECT_LENGTH,vertices);
                    int bkdLastIndex = invWorkIndex;

                    for(int negtiveIndex = invWorkIndex - 1; negtiveIndex >= 0; negtiveIndex--)
                    {
                        if(true == judgePointInPnPoly(vertices,fitted_points[negtiveIndex](0),fitted_points[negtiveIndex](1)))
                        {
                            if(0 == invSegmentID)
                            {
                                backward_part_indice.push_back(negtiveIndex);
                            }
                            invWorkdIndice.push_back(negtiveIndex);
                            if(negtiveIndex < bkdLastIndex)
                            {
                                bkdLastIndex = negtiveIndex;
                            }
                        }
                        else
                        {
                            break;
                        }
                    }
                    if(bkdLastIndex >= 0)
                    {
                        if(bkdLastIndex < SEG_LINE_MIN_POINTS)
                        {//terminated
                            for(int negtiveIndex = bkdLastIndex-1; negtiveIndex >= 0; negtiveIndex--)
                            {
                                invWorkdIndice.push_back(negtiveIndex);
                                if(0 == invSegmentID)
                                {
                                    backward_part_indice.push_back(negtiveIndex);
                                }
                            }
                            invWorkIndex = -1;  
                        }
                        else
                        {
                            invWorkIndex = bkdLastIndex - 1;
                        }
                        if(invSegmentID > 0)
                        {
                            backward_segmented_indice.push_back(invWorkdIndice);
                        }
                        
                        //next segment
                        invSegmentID++;
                        std::vector<size_t>().swap(invWorkdIndice);
                    }
                    else
                    {
                        backward_segmented_indice.push_back(invWorkdIndice);
                        
                        //next segment
                        invSegmentID++;
                        invWorkIndex--;
                        std::vector<size_t>().swap(invWorkdIndice);
                    }
                }
                //merging all segments
                //backward segments
                std::vector<Point2DF> rear_all_points;
                std::vector<size_t> previous_rear_small_seg;
                int rear_parts_num = static_cast<int>(backward_segmented_indice.size());
                for(int bkdIdx = rear_parts_num - 1; bkdIdx >= 0; bkdIdx--)
                {
                    std::vector<size_t> rear_line_indice;
                    std::vector<Point2DF> rear_segment_points;
                    
                    if(previous_rear_small_seg.size() > 0 )
                    {//previous part
                        for(auto &pt:previous_rear_small_seg)
                        {
                            rear_line_indice.push_back(pt);
                        }
                        std::vector<size_t>().swap(previous_rear_small_seg);
                    }
                    for(int i = static_cast<int>(backward_segmented_indice[bkdIdx].size() - 1) ;i >= 0;i--)
                    {
                        rear_line_indice.push_back(backward_segmented_indice[bkdIdx][i]);                
                    }

                    if(rear_line_indice.size() > SEG_LINE_MIN_POINTS)
                    {
                        for(size_t idx = 0; idx < rear_line_indice.size(); idx++)
                        {
                            Point2DF pts;
                            pts.x = fitted_points[rear_line_indice[idx]](0);
                            pts.y = fitted_points[rear_line_indice[idx]](1);
                            rear_segment_points.push_back(pts);
                        }
                        seg_groups.push_back(rear_segment_points);
                    }
                    else
                    {
                        previous_rear_small_seg.swap(rear_line_indice);
                    }
                }
                if(previous_rear_small_seg.size() > 0 || backward_part_indice.size() > 0)
                {
                    std::vector<size_t> nearest_rear_seg_indice;
                    if(previous_rear_small_seg.size() > 0 )
                    {//previous part
                        for(auto &pt:previous_rear_small_seg)
                        {
                            nearest_rear_seg_indice.push_back(pt);
                        }
                    }
                    for(int i = static_cast<int>(backward_part_indice.size() - 1) ;i >= 0;i--)
                    {
                        nearest_rear_seg_indice.push_back(backward_part_indice[i]);
                    }
                    std::vector<Point2DF> nearest_rear_seg_points;
                    for(auto &ptIdx: nearest_rear_seg_indice)
                    {
                        Point2DF pts;
                        pts.x = fitted_points[ptIdx](0);
                        pts.y = fitted_points[ptIdx](1);
                        nearest_rear_seg_points.push_back(pts);
                    }
                    if(nearest_rear_seg_points.size() > 0)
                    {
                        seg_groups.push_back(nearest_rear_seg_points);
                    }
                }

                //The first front part
                std::vector<size_t> first_front_line_indice;
                std::vector<Point2DF> first_front_segment_points;
                for(size_t i = 0 ;i < forward_part_indice.size();i++)
                {
                    first_front_line_indice.push_back(forward_part_indice[i]);
                    Point2DF pts;
                    pts.x = fitted_points[forward_part_indice[i]](0);
                    pts.y = fitted_points[forward_part_indice[i]](1);
                    first_front_segment_points.push_back(pts);
                }

                //forward segments
                std::vector<Point2DF> previous_front_small_part;
                size_t FRIST_FONT_SIZE = first_front_segment_points.size();
                if(FRIST_FONT_SIZE >= SEG_LINE_MIN_POINTS)
                {
                    seg_groups.push_back(first_front_segment_points);
                }
                else if(FRIST_FONT_SIZE > 0)
                {
                    float delta_x = first_front_segment_points[FRIST_FONT_SIZE-1].x - first_front_segment_points[0].x;
                    if(fabs(delta_x) >= FIRST_FRONT_SEGMENT_LENGTH)
                    {
                        seg_groups.push_back(first_front_segment_points);
                    }
                    else
                    {
                        previous_front_small_part.swap(first_front_segment_points);
                    }
                }
                for(size_t fwdIdx = 0 ; fwdIdx < forward_segmented_indice.size();fwdIdx++)
                {

                    std::vector<Point2DF> front_segment_points;
                    if(previous_front_small_part.size() > 0 )
                    {//previous part
                        for(auto &pt:previous_front_small_part)
                        {
                            front_segment_points.push_back(pt);
                        }
                        std::vector<Point2DF>().swap(previous_front_small_part);
                    }
                    for(size_t i = 0; i < forward_segmented_indice[fwdIdx].size(); i++)
                    {
                        Point2DF pts;
                        pts.x = fitted_points[forward_segmented_indice[fwdIdx][i]](0);
                        pts.y = fitted_points[forward_segmented_indice[fwdIdx][i]](1);
                        front_segment_points.push_back(pts);
                    }
                    if(front_segment_points.size() >= SEG_LINE_MIN_POINTS)
                    {
                        seg_groups.push_back(front_segment_points);
                    }
                    else
                    {
                        previous_front_small_part.swap(front_segment_points);
                    }
                }
                if(previous_front_small_part.size() > 0)
                {
                    if(seg_groups.size() > 0 )
                    {
                        int size = seg_groups.size();
                        for(auto &pt: previous_front_small_part)
                        {
                            seg_groups[size-1].emplace_back(pt);
                        }
                    }
                    else
                    {
                        seg_groups.push_back(previous_front_small_part);
                    }
                }
            }
            else if(fitted_points.size() >= SEG_LINE_MIN_POINTS/2)
            {
                std::vector<Point2DF> oneSeg;
                for(size_t i = 0; i < fitted_points.size(); i++)
                {
                    Point2DF pts;
                    pts.x = fitted_points[i](0);
                    pts.y = fitted_points[i](1);
                    oneSeg.push_back(pts);
                }
                seg_groups.push_back(oneSeg);
            }
        }
        else
        {
            std::vector<Point2DF> oneSeg;
            for(size_t i = 0; i < raw_points.size(); i++)
            {
                Point2DF pts;
                pts.x = raw_points[i](0);
                pts.y = raw_points[i](1);
                oneSeg.push_back(pts);
            }
            seg_groups.push_back(oneSeg);
        }
    }

    return;
}

void FitRawCurveLineWithCoeffs(
    const std::vector<Point2DF> &raw_line_points, 
    const bool & removed_invalid,
    std::vector<Curve> & fit_curves,
    std::vector<std::vector<Point2DF> > & bev_seg_groups)
{
    std::vector<std::vector<Point2DF>> raw_segment_group;
    GetSegsByBSplineAndRectangle(raw_line_points,raw_segment_group);
	
    const float LINE_SEGMENT_LENGTH = 50.0;
    const float FITTED_INTERVAL_DISTANCE = 0.1;
    const float scaled_parameter = 0.001;
    typedef struct index_data_s
    {
        int idx;
        std::vector<Point2DF> data;
    }index_data;

    int first_front_index = -1;
    bool index_found = false;
    bool use_rear_point = false;
    bool use_front_point = false;
    for(size_t line_index = 0; !index_found && line_index < raw_segment_group.size();line_index++)
    {
        for(size_t point_index = 0; !index_found && point_index < raw_segment_group[line_index].size(); point_index++)
        {
            if(raw_segment_group[line_index][point_index].x >= 0.0)
            {
                first_front_index = static_cast<int>(line_index);
                index_found = true;
            }
        }
    }

    for(int seg_idx = 0; seg_idx < raw_segment_group.size(); seg_idx++)
    {    
        auto &bevseg = raw_segment_group[seg_idx];
        uint32_t rows_num = bevseg.size();
        uint32_t cols_num = 4;
        std::shared_ptr<LeastSquaresSolver> solver(new LeastSquaresSolver);

        if (cols_num > 0 && rows_num > cols_num)
        {
            // least square solver initialization
            solver->InitializeSolver(rows_num, cols_num);
            solver->SetRegularizedParam(1e-5);

            // set measurement points
            int32_t point_index = 0;
            std::vector<index_data> point_segments;
            for (size_t i = 0; i < rows_num; ++i)
            {
                if(first_front_index > 0 && seg_idx == first_front_index && false == use_rear_point)
                {//For smoothening,add the first point in the back of car
                    size_t rear_first_size = raw_segment_group[first_front_index-1].size();
                    if(rear_first_size > 0)
                    {
                        float rear_point_xs = raw_segment_group[first_front_index-1][rear_first_size-1].x * scaled_parameter;
                        float rear_point_ys = raw_segment_group[first_front_index-1][rear_first_size-1].y * scaled_parameter;
                        solver->AddElementToA(point_index, 0, scaled_parameter);
                        solver->AddElementToA(point_index, 1, rear_point_xs);
                        solver->AddElementToA(point_index, 2, rear_point_xs * rear_point_xs);
                        solver->AddElementToA(point_index, 3, rear_point_xs * rear_point_xs * rear_point_xs);

                        solver->AddElementToB(point_index, rear_point_ys);
                        ++point_index;
                        use_rear_point = true;
                        continue;
                    }
                }

                float point_xs = bevseg[i].x * scaled_parameter;
                float point_ys = bevseg[i].y * scaled_parameter;
                solver->AddElementToA(point_index, 0, scaled_parameter);
                solver->AddElementToA(point_index, 1, point_xs);
                solver->AddElementToA(point_index, 2, point_xs * point_xs);
                solver->AddElementToA(point_index, 3, point_xs * point_xs * point_xs);

                solver->AddElementToB(point_index, point_ys);
                ++point_index;

                if(bevseg[i].x > 4.0*LINE_SEGMENT_LENGTH)
                {
                    continue;
                }

                int segIdx = static_cast<int>(std::floor(bevseg[i].x/ LINE_SEGMENT_LENGTH));
                bool exist = false;
                for(size_t k = 0 ;k < point_segments.size();k++)
                {
                    if(point_segments[k].idx == segIdx)
                    {
                        exist = true;
                        point_segments[k].data.emplace_back(bevseg[i]);
                        break;
                    }
                }

                if(false == exist)
                {
                    index_data id;
                    id.idx = segIdx;
                    id.data.emplace_back(bevseg[i]);
                    point_segments.emplace_back(id);
                }
            }

            if(first_front_index > 0 && seg_idx == (first_front_index-1))
            {//For smoothening,add the first point in the first front of car
                size_t front_first_size = raw_segment_group[first_front_index].size();
                if(front_first_size > 0)
                {
                    float front_point_xs = raw_segment_group[first_front_index-1][0].x * scaled_parameter;
                    float front_point_ys = raw_segment_group[first_front_index-1][0].y * scaled_parameter;
                    solver->AddElementToA(point_index, 0, scaled_parameter);
                    solver->AddElementToA(point_index, 1, front_point_xs);
                    solver->AddElementToA(point_index, 2, front_point_xs * front_point_xs);
                    solver->AddElementToA(point_index, 3, front_point_xs * front_point_xs * front_point_xs);

                    solver->AddElementToB(point_index, front_point_ys);
                    ++point_index;
                }
            }
            solver->AddElementToU(2, 1e-7); // c2

            // solve
            solver->CalculateX();

            Curve fit_curve;
            fit_curve.c0 = solver->GetX(0);
            fit_curve.c1 = solver->GetX(1);
            fit_curve.c2 = solver->GetX(2);
            fit_curve.c3 = solver->GetX(3);

            fit_curve.c2 *= scaled_parameter;
            fit_curve.c3 *= (scaled_parameter * scaled_parameter);
            fit_curve.lon_dist_start = bevseg.front().x;
            fit_curve.lon_dist_end = bevseg.back().x;

            float need_fit_again_count  = 0;
            std::vector<SamplingPoint> new_line_points;
            for(int n = 0 ;n < rows_num;n++)
            {
                Point2DF curr_point = bevseg[n];
                float calc_y = CalculateY(fit_curve.c0, fit_curve.c1, fit_curve.c2,fit_curve.c3, curr_point.x);
                Point2DF fitted_point(curr_point.x,calc_y);
                float dist = CalculatePoint2PointDist(curr_point,fitted_point);

                if(n < (rows_num - 1))
                {
                    SamplingPoint pts;
                    pts.x = curr_point.x;
                    pts.y = curr_point.y;
                    new_line_points.emplace_back(pts);
                }

                Point2DF next_point;
                bool next_point_exist = false;
                if(dist >= FITTED_INTERVAL_DISTANCE && n < (rows_num - 1))
                {
                    need_fit_again_count++;
                    next_point = bevseg[n+1];
                    next_point_exist = true;
                }
                else if(dist >= FITTED_INTERVAL_DISTANCE && n == (rows_num - 1) && (n - 1) >= 0)
                {
                    curr_point = bevseg[n-1];
                    next_point = bevseg[n];
                    next_point_exist = true;
                }
                if(true == next_point_exist)
                {
                    float new_num_pts = static_cast<int>(std::floor(dist*10));
                    //considering small turning radius,e.g. HDMAP,interval 20cm.
                    new_num_pts = (new_num_pts >= 10) ? 10:new_num_pts;
                    float length = CalculatePoint2PointDist(curr_point,next_point);
                    float step = length/(new_num_pts+1);
                    Eigen::Vector2f vec,unitVec;
                    vec(0) = next_point.x - curr_point.x;
                    vec(1) = next_point.y - curr_point.y;
                    if(length > 0.0)
                    {
                        unitVec = vec/length;
                        int index = 1;
                        while(index*step < length)
                        {
                            SamplingPoint new_point;
                            new_point.x = curr_point.x + index*step*unitVec(0);
                            new_point.y = curr_point.y + index*step*unitVec(1);
                            new_line_points.emplace_back(new_point);
                            index++;
                        }
                    }
                }
                if(n == (rows_num - 1) )
                {//last point
                    SamplingPoint pts;
                    pts.x = bevseg[n].x;
                    pts.y = bevseg[n].y;
                    new_line_points.emplace_back(pts);
                }
            }
            if(need_fit_again_count > 0)
            {
                Curve new_curve;
                bool res = FitterLeastSquareMethod(new_line_points,3,new_curve);
                if(true == res)
                {
                    fit_curve.c0 = new_curve.c0;
                    fit_curve.c1 = new_curve.c1;
                    fit_curve.c2 = new_curve.c2;
                    fit_curve.c3 = new_curve.c3;      
                }
            }
            //merge less than 2 points segment into prev/next segment.
            std::set<size_t> merge_indice;
            for(size_t d = 0; d < point_segments.size(); d++)
            {
                if(point_segments[d].data.size() < 2)
                {
                    merge_indice.insert(d);
                }
            }
            for(auto & index: merge_indice)
            {
                if(index > 0 && point_segments[index].data.size() > 0)
                {
                    point_segments[index-1].data.emplace_back(point_segments[index].data[0]);
                }
            }
            int new_seg_num = point_segments.size();
            if(new_seg_num > 0)
            {
                for(size_t p = 0; p < point_segments.size(); p++)
                {
                    std::vector<Point2DF> new_seg;
                    for(auto &pt: point_segments[p].data)
                    {
                        new_seg.emplace_back(pt);
                    }

                    if(new_seg.size() > 1)
                    {
                        bev_seg_groups.emplace_back(new_seg);
                        //copy curve fit
                        Curve seg_fit_curve;
                        seg_fit_curve.c0 = fit_curve.c0;
                        seg_fit_curve.c1 = fit_curve.c1;
                        seg_fit_curve.c2 = fit_curve.c2;
                        seg_fit_curve.c3 = fit_curve.c3;
                        if(fabs(fit_curve.c2) >= 0.5 || fabs(fit_curve.c0) >= LINE_SEGMENT_LENGTH)
                        {//exception!
                            Curve new_curve;
                            std::vector<SamplingPoint> new_to_fit_points;
                            for(auto &pts: new_seg)
                            {
                                SamplingPoint pt;
                                pt.x = pts.x;
                                pt.y = pts.y;
                                new_to_fit_points.emplace_back(pt);
                            }
                            seg_fit_curve.c0 = 0;
                            seg_fit_curve.c1 = 0;
                            seg_fit_curve.c2 = 0;
                            seg_fit_curve.c3 = 0;
                            bool res = FitterLeastSquareMethod(new_to_fit_points,3,new_curve);
                            if(true == res)
                            {
                                seg_fit_curve.c0 = new_curve.c0;
                                seg_fit_curve.c1 = new_curve.c1;
                                seg_fit_curve.c2 = new_curve.c2;
                                seg_fit_curve.c3 = new_curve.c3;
                            }
                        }
                        seg_fit_curve.lon_dist_start = new_seg.front().x;
                        seg_fit_curve.lon_dist_end = new_seg.back().x;
                        fit_curves.emplace_back(seg_fit_curve);
                    }
                }
            }
        }
        else if(false == removed_invalid && rows_num >= 2)
        {
            Curve fit_curve;
            std::vector<SamplingPoint> new_to_fit_points;
            for(auto &pts: bevseg)
            {
                SamplingPoint pt;
                pt.x = pts.x;
                pt.y = pts.y;
                new_to_fit_points.emplace_back(pt);
            }
            bool res = FitterLeastSquareMethod(new_to_fit_points,3,fit_curve);
            if(true == res)
            {
                fit_curve.lon_dist_start = bevseg.front().x;
                fit_curve.lon_dist_end = bevseg.back().x;
                fit_curves.emplace_back(fit_curve);
                bev_seg_groups.emplace_back(bevseg);
            }
        }
    }

    return;
}



} // namespace fusion
} // namespace cem
