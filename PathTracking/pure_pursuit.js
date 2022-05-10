// pure pursuit path tracker
// ref: https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathTracking/pure_pursuit

class PurePursuit{
    constructor() {
        this.k = 0.1 // look forward gain
        this.Lfc = 50.0 // lookg-ahead distance
        this.Kp = 1.0 // speed proportional gain
        this.old_nearest_point_index = -1;
        this.current_idx = -1;    
    }

    reset() {
        this.old_nearest_point_index = -1;
        this.current_idx = -1; 
    }

    set_lookahead(lookahead) {
        this.Lfc = lookahead;
    }

    search_target_index(car, path) {
        let ind = 0;
        if (this.old_nearest_point_index == -1) {
            let d = [];
            for (let i in path.x) {
                d.push(car.calc_distance_to_point(new Vector2D(path.x[i], path.y[i])));
            }
            ind = d.indexOf(Math.min(...d));
            this.old_nearest_point_index = ind;
        }    
        else {
            ind = this.old_nearest_point_index;
            let dist_to_this_index = car.calc_distance_to_point(new Vector2D(path.x[ind], path.y[ind]));
            while (ind < path.x.length) {
                let dist_to_next_index = car.calc_distance_to_point(new Vector2D(path.x[ind + 1], path.y[ind + 1]));
                if (dist_to_this_index < dist_to_next_index) {
                    break;
                }
                // ind = (((ind + 1) < path.x.length) ? ind + 1 : ind);
                ind++;
                dist_to_this_index = dist_to_next_index;
            }
            this.old_nearest_point_index = ind;
        }
        let Lf = this.k * car.speed + this.Lfc;
        if (ind == -1) {
            return [0, Lf];
        }
        while (Lf > car.calc_distance_to_point(new Vector2D(path.x[ind], path.y[ind]))) {
            // console.log("while 2");
            if ((ind + 1) >= path.x.length) {
                break;
            }
            ind += 1;
        }
        return [ind, Lf];
    }

    proportional_control(target, current) {
        return this.Kp * (target - current);
    }
    
    compute_cmd(car, path) {
        var [index, lookforward] = this.search_target_index(car, path);
        let arrived = false;
        if (this.current_idx >= index) {
            index = this.current_idx;
        }
        var tx = 0;
        var ty = 0;
        var direction = 1;
        if (index < path.x.length) {
            tx = path.x[index];
            ty = path.y[index];
            direction = path.directions[index];
        }
        else {
            tx = path.x[path.x.length - 1];
            ty = path.y[path.y.length - 1];
            direction = path.directions[path.directions.length - 1];
        }
    
        let alpha = Math.atan2(ty - car.rear_y, tx - car.rear_x) - car.yaw;
        let delta = Math.atan2(2.0 * car.wheel_base * Math.sin(alpha) / lookforward, 1.0);
        let pedal = direction * this.proportional_control(car.max_speed, car.speed);
        let dist_to_goal = car.calc_distance_to_point(new Vector2D(path.x[path.x.length - 1], path.y[path.y.length - 1]));
        if (dist_to_goal < 10) {
            arrived = true;
        }
        this.current_idx = index;
        return [pedal, delta, arrived];
    }
}