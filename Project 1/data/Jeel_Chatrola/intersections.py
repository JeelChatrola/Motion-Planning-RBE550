import time

def segment_pair_intersection(line1, line2):
    '''Return the intersection point between 2 line segments or None, if they do not intersect. 

    arguments:
    line1 - is the first line segment and has the following form ((x1,y1), (x2,y2)) where x1, y1 is
      the first point of the line segment and x2,y2 is the second point of the line segment.
    line2 - is the second line segment and has the same format as line1.

    return:
    ipoint - A tuple (x,y) which is the point of intersection or None if it does not exist 
    '''
    
    ### YOUR CODE STARTS HERE ###
    intersect = True 

    if intersect:
        # intersection point between line1 and line2
        
        # vector L1 or A1B1 ( x2-x1 , y2-y1 )
        A1 = (line1[0][0],line1[0][1])
        B1 = (line1[1][0],line1[1][1])
        vec_L1 = (B1[0] - A1[0] , B1[1] - A1[1])
        # print('Vector A1B1 or a1 is',vec_L1)

        # vector L1 or A2B2 ( x2-x1 , y2-y1 )
        A2 = (line2[0][0],line2[0][1])
        B2 = (line2[1][0],line2[1][1])
        vec_L2 = (B2[0] - A2[0], B2[1] - A2[1])
        # print('Vector A2B2 or a2 is',vec_L2)
        
        # A2 - A1 
        A2A1 = ((A2[0] - A1[0],A2[1] - A1[1])) 
        # print("A2A1",A2A1)

        # A1 - A2 
        A1A2 = ((A1[0] - A2[0],A1[1] - A2[1])) 
        # print("A2A1",A1A2)

        # Perpendicular or L2 = ( - ay,ax )
        L2_perp = ( -vec_L2[1],vec_L2[0])
        # print('L2 perpendicular or a2T',L2_perp)

        # Perpendicular of L1 = ( - ay,ax )
        L1_perp = ( -vec_L1[1],vec_L1[0])
        # print('L1 perpendicular or a1T',L1_perp)
        
        if (vec_L2[0] != 0 or vec_L1[0] != 0):        

            if( (L2_perp[0]*vec_L1[0] + L2_perp[1]*vec_L1[1])!=0 and (L2_perp[0]*vec_L1[0] + L2_perp[1]*vec_L1[1])!=0 ) :
    
                S = (L2_perp[0]*A2A1[0] + L2_perp[1]*A2A1[1])/ (L2_perp[0]*vec_L1[0] + L2_perp[1]*vec_L1[1])
                # print("S is ",S)
    
                T = (L1_perp[0]*A1A2[0] + L1_perp[1]*A1A2[1])/ (L1_perp[0]*vec_L2[0] + L1_perp[1]*vec_L2[1])
                # print("T is ",T)
    
                if (0 <= S <= 1) and (0 <= T <= 1 ):
                    P = (A1[0] + S*vec_L1[0],A1[1] + S*vec_L1[1]) 
                    return (P[0],P[1])

        return None        
    else:
        return None
    ### YOUR CODE ENDS HERE ###


def efficient_intersections(L1, L2):
    #This is the Bonus part of the assignment 

    ### YOUR CODE STARTS HERE ###

    # Return Empty if zero elements
    if ( len(L1) <= 0 ) and ( len(L2) <= 0 ):
        return ([], [])

    # Converted to a list of list from list of tuple of tuple        
    flattened_list = [list(item) for tup in L1 for item in tup]

    # Sort the array based on x-coordinates
    x_coord_sorted = sorted(flattened_list , key=lambda k: k[0])

    # Event Queue - ( 'segment start', 'segment end', 'segment cross' ) - insert,findmin, deletemin
    # event_flag = ['ss','se','sc']

    # Segment List - segment i = left endpoint, right endpoint and mi (slope) and bi (y-intercept)  
    sweep_line_start = x_coord_sorted[0]

    # Creating a dictionary to store the line segments that are currently intersecting the sweep line.
    # intersect_segments = {1:'sweep_line_start'}

    # for i in range(1,len(x_coord_sorted)):

    return ([], []) 
    ### YOUR CODE ENDS HERE ###


def all_pairs_intersections(L1, L2):
    x = []
    y = []
    for l1 in L1: 
        for l2 in L2: 
          point =  segment_pair_intersection(l1, l2)
          if point: 
            x.append(point[0])
            y.append(point[1])

    return (x,y)

