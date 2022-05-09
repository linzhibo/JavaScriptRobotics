// classes
class Car {
    constructor(context, x, y, color, queue = 40) {
        this.context = context;
        this.car_width = 25;
        this.car_length = 50;
        this.wheel_base = 30;
        this.max_speed = 20;
        this.max_steer = 0.5; //~30 degrees
        this.history_points = [];
        this.history_points_queue = queue;

        this.x = x;
        this.y = y;
        this.yaw = 0.0;
        this.speed = 0.0;
        this.rear_x = this.x - ((this.wheel_base / 2) * Math.cos(this.yaw));
        this.rear_y = this.y - ((this.wheel_base / 2) * Math.sin(this.yaw));
        this.color = color;

        this.render();
    }

    set_max_speed(speed) { this.max_speed = speed; }
    
    set_max_steer(steer) { this.max_steer = steer; }
    
    reset_speed(){
        this.speed = 0.0;
    }
    reset_history_points() {
        this.history_points = [];
    }

    update(pedal, steer, dt, brake = false) {
        steer = clamp(steer, -this.max_steer, this.max_steer);
        this.x += this.speed * Math.cos(this.yaw) * dt;
        this.y += this.speed * Math.sin(this.yaw) * dt;
        this.yaw += this.speed / this.wheel_base * Math.tan(steer) * dt;
        this.yaw = normalize_angle(this.yaw);
        this.speed += pedal * dt;
        this.speed = clamp(this.speed, -this.max_speed, this.max_speed);
        if (brake) {
            this.reset_speed();
        }
        this.rear_x = this.x - ((this.wheel_base / 2) * Math.cos(this.yaw));
        this.rear_y = this.y - ((this.wheel_base / 2) * Math.sin(this.yaw));
        if (this.history_points.length < 2) {
            this.history_points.push(new Vector2D(this.x, this.y));
        }
        else {
            let new_pt = new Vector2D(this.x, this.y);
            let last_pt = this.history_points[this.history_points.length - 1];
            let dist_to_revious_pt = Math.hypot(last_pt.x - new_pt.x, last_pt.y - new_pt.y);
            if (dist_to_revious_pt > 10){
                this.history_points.push(new_pt);
                if (this.history_points.length > this.history_points_queue) {
                    this.history_points.shift();
                }
            }
        }
    }

    calc_distance_to_point(point) {
        return Math.hypot(this.rear_x - point.x, this.rear_y - point.y);
    }
    
    render() {
        this.context.beginPath();
        this.context.strokeStyle = "orange";
        for (let i in this.history_points) {
            this.context.fillRect(this.history_points[i].x, this.history_points[i].y, 2, 2);
        }
        this.context.strokeStyle = this.color;
        this.context.lineWidth = 2;
        //transformation
        this.context.translate(this.x, this.y);
        this.context.rotate(this.yaw);
        this.context.translate(-this.x, -this.y)
        this.context.rect(this.x - this.car_length/2, this.y - this.car_width/2, this.car_length, this.car_width);
        this.context.setTransform(1, 0, 0, 1, 0, 0);
        this.context.stroke();
    }
};  