''' 
    Given corners of a polygon and a point,
    find whether the point is inside polygon
'''

def slope(x1,y1,x2,y2):
    ''' find slope of line through two points'''
    num=float(y2-y1)
    denom=x2-x1
    if denom == 0:
        return float('inf')
    return num/denom

def pointSlopeLine(x, point, slope):
    '''
        returns the right half of point slope
        line equation with given x, point, and slope
    '''
    X = 0
    Y = 1
    return slope*(x-point[X])+point[Y]
    
def isInPoly(Corners, Point):
    '''
        Point in form [x,y]
        Corners is a list of points marking the corners
        of the polygon

        returns boolean of whether of not the point is in the polygon
    '''

    X = 0
    Y = 1

    for i in range(len(Corners)):
        c1 = Corners[i-1]
        c2 = Corners[i]
        m = slope(c1[X],c1[Y],c2[X],c2[Y])

        Test = Corners[i-2]
        Test = Corners[i-2]
        if Test[Y] > pointSlopeLine(Test[X], c1, m):
            if Point[Y] < pointSlopeLine(Point[X], c1, m):
                return False
        elif Point[Y] > pointSlopeLine(Point[X], c1, m):
            return False
    return True


if __name__=='__main__':
    Corners = [[0,0],[0,2],[10,15],[12,12],[7,-1]]
    Point = [10, 16]
    print isInPoly(Corners, Point)