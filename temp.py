 #Test if either points are inside the circle
    for circle in circles:
        
        dist1 = calc_dist(q_near, circle.center)
        dist2 = calc_dist(q_new, circle.center)
        
        u = find_u(q_near, q_new, circle.center)
        
        if u >= 0 and u <= 1:
            # Compute the coordinates of the closest point
            closest_x = q_near[0] + u * (q_new[0] - q_near[0])
            closest_y = q_near[1] + u * (q_new[1] - q_near[1])
            closest_point = (closest_x, closest_y)
            
            # Distance from the closest point to the circle center
            dist3 = calc_dist(closest_point, circle.center)
            
            # Check if the closest point is inside the circle
            if dist3 < circle.radius:
                collision = True
                break  # Exit the loop if a collision is detected
        else:
            # Check if q_near or q_new is inside the circle
            if dist1 < circle.radius or dist2 < circle.radius:
                collision = True
                break