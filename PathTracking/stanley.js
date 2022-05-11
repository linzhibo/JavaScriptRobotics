
class StanleyController{
    constructor(){
        this.k = 0.5;  // control gain
        this.Kp = 1.0;  // speed proportional gain
        this.last_target_index = -1;
        this.old_nearest_point_index = -1;
    }

    reset() {
        this.last_target_index = -1;
    }

    // calc_target_index(car, path){
    //     let fx = car.x + car.wheel_base * Math.cos(car.yaw);
    //     let fy = car.y + car.wheel_base * Math.sin(car.yaw);
    //     let d=[], dx=[], dy =[];
    //     for (let i in path.x){
    //         dx.push(fx - path.x[i]);
    //         dy.push(fy - path.y[i]);
    //         d.push(Math.hypot(fx - path.x[i], fy - path.y[i]));
    //     }
    //     let target_index = d.indexOf(Math.min(...d));
    //     let front_axle_vec = [-Math.cos(car.yaw + Math.PI/2), -Math.sin(car.yaw + Math.PI/2)];
    //     let error_front_axle = dx[target_index] * front_axle_vec[0] + dy[target_index] * front_axle_vec[1];
    //     return [target_index, error_front_axle];
    // }

    calc_target_index(car, path) {
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
        let Lf = this.k * car.speed + 20;
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
        let fx = car.x + car.wheel_base * Math.cos(car.yaw);
        let fy = car.y + car.wheel_base * Math.sin(car.yaw);
        let d=[], dx=[], dy =[];
        for (let i in path.x){
            dx.push(fx - path.x[i]);
            dy.push(fy - path.y[i]);
            d.push(Math.hypot(fx - path.x[i], fy - path.y[i]));
        }
        let front_axle_vec = [-Math.cos(car.yaw + Math.PI/2), -Math.sin(car.yaw + Math.PI/2)];
        let error_front_axle = dx[ind] * front_axle_vec[0] + dy[ind] * front_axle_vec[1];
        return [ind, error_front_axle];
    }

    proportional_control(target, current) {
        return this.Kp * (target - current);
    }

    compute_cmd(car, path){
        let [current_target_index, error_front_axle] = this.calc_target_index(car, path);
        if (this.last_target_index >= current_target_index){
            current_target_index = this.last_target_index;
        }
        let theta_e = normalize_angle(path.yaw[current_target_index] - car.yaw);
        let theta_d = Math.atan2(this.k * error_front_axle, car.speed);
        let delta = theta_e + theta_d;
        let direction = path.directions[current_target_index];
        let pedal = direction * this.proportional_control(car.max_speed, car.speed);
        let arrived = false;
        let dist_to_goal = car.calc_distance_to_point(new Vector2D(path.x[path.x.length - 1], path.y[path.y.length - 1]));
        if (dist_to_goal < 10) {
            arrived = true;
        }
        this.last_target_index = current_target_index;
        return [pedal, delta, arrived];
    }
}