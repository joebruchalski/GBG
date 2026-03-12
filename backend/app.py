"""
Main Flask Application for GBG Field Service API
"""

import os
from flask import Flask, jsonify, request
from flask_cors import CORS
from flask_sqlalchemy import SQLAlchemy
from datetime import datetime
import logging

from optimization.scheduler import (
    RouteOptimizer, ServiceCall, Technician, Location, 
    TimeWindow, Priority, SkillType, OptimizationConfig
)

# Initialize Flask app
app = Flask(__name__)
app.config['SECRET_KEY'] = os.environ.get('SECRET_KEY', 'dev-secret-key')
app.config['SQLALCHEMY_DATABASE_URI'] = os.environ.get(
    'DATABASE_URL', 
    'sqlite:///gbg_field_service.db'
)
app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

# Initialize extensions
db = SQLAlchemy(app)
CORS(app, resources={r"/api/*": {"origins": "*"}})

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize optimizer
optimizer = RouteOptimizer()


# Database Models
class ServiceCallModel(db.Model):
    __tablename__ = 'service_calls'
    
    id = db.Column(db.String(50), primary_key=True)
    customer_name = db.Column(db.String(100), nullable=False)
    address = db.Column(db.String(200), nullable=False)
    latitude = db.Column(db.Float, nullable=False)
    longitude = db.Column(db.Float, nullable=False)
    scheduled_date = db.Column(db.Date, nullable=False)
    time_window_start = db.Column(db.Time, nullable=False)
    time_window_end = db.Column(db.Time, nullable=False)
    duration_minutes = db.Column(db.Integer, default=60)
    priority = db.Column(db.String(20), default='MEDIUM')
    required_skills = db.Column(db.String(200))  # JSON string
    status = db.Column(db.String(20), default='pending')
    assigned_technician_id = db.Column(db.String(50))
    notes = db.Column(db.Text)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)
    updated_at = db.Column(db.DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)


class TechnicianModel(db.Model):
    __tablename__ = 'technicians'
    
    id = db.Column(db.String(50), primary_key=True)
    name = db.Column(db.String(100), nullable=False)
    email = db.Column(db.String(100), unique=True)
    phone = db.Column(db.String(20))
    skills = db.Column(db.String(200))  # JSON string
    home_address = db.Column(db.String(200))
    home_latitude = db.Column(db.Float)
    home_longitude = db.Column(db.Float)
    work_start_time = db.Column(db.Time, default=datetime.strptime('08:00', '%H:%M').time())
    work_end_time = db.Column(db.Time, default=datetime.strptime('17:00', '%H:%M').time())
    max_calls_per_day = db.Column(db.Integer, default=8)
    active = db.Column(db.Boolean, default=True)
    created_at = db.Column(db.DateTime, default=datetime.utcnow)


# API Routes
@app.route('/api/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        'status': 'healthy',
        'timestamp': datetime.utcnow().isoformat()
    })


@app.route('/api/dashboard/stats', methods=['GET'])
def get_dashboard_stats():
    """Get dashboard statistics"""
    try:
        today = datetime.utcnow().date()
        
        total_calls = ServiceCallModel.query.count()
        completed_today = ServiceCallModel.query.filter_by(
            scheduled_date=today,
            status='completed'
        ).count()
        scheduled_today = ServiceCallModel.query.filter_by(
            scheduled_date=today
        ).count()
        active_technicians = TechnicianModel.query.filter_by(active=True).count()
        
        return jsonify({
            'totalCalls': total_calls,
            'completedCalls': completed_today,
            'scheduledToday': scheduled_today,
            'activeTechnicians': active_technicians
        })
    except Exception as e:
        logger.error(f"Error getting dashboard stats: {e}")
        return jsonify({'error': 'Failed to fetch statistics'}), 500


@app.route('/api/service-calls', methods=['GET'])
def get_service_calls():
    """Get all service calls"""
    try:
        calls = ServiceCallModel.query.all()
        return jsonify([{
            'id': call.id,
            'customerName': call.customer_name,
            'address': call.address,
            'scheduledDate': call.scheduled_date.isoformat(),
            'timeWindow': f"{call.time_window_start.strftime('%H:%M')} - {call.time_window_end.strftime('%H:%M')}",
            'priority': call.priority,
            'status': call.status,
            'assignedTechnician': call.assigned_technician_id
        } for call in calls])
    except Exception as e:
        logger.error(f"Error fetching service calls: {e}")
        return jsonify({'error': 'Failed to fetch service calls'}), 500


@app.route('/api/service-calls', methods=['POST'])
def create_service_call():
    """Create a new service call"""
    try:
        data = request.json
        
        # Generate ID if not provided
        if 'id' not in data:
            data['id'] = f"SC{datetime.utcnow().strftime('%Y%m%d%H%M%S')}"
        
        call = ServiceCallModel(
            id=data['id'],
            customer_name=data['customerName'],
            address=data['address'],
            latitude=data['latitude'],
            longitude=data['longitude'],
            scheduled_date=datetime.fromisoformat(data['scheduledDate']).date(),
            time_window_start=datetime.strptime(data['timeWindowStart'], '%H:%M').time(),
            time_window_end=datetime.strptime(data['timeWindowEnd'], '%H:%M').time(),
            duration_minutes=data.get('durationMinutes', 60),
            priority=data.get('priority', 'MEDIUM'),
            required_skills=','.join(data.get('requiredSkills', [])),
            notes=data.get('notes', '')
        )
        
        db.session.add(call)
        db.session.commit()
        
        return jsonify({
            'id': call.id,
            'message': 'Service call created successfully'
        }), 201
        
    except Exception as e:
        logger.error(f"Error creating service call: {e}")
        db.session.rollback()
        return jsonify({'error': 'Failed to create service call'}), 500


@app.route('/api/technicians', methods=['GET'])
def get_technicians():
    """Get all technicians"""
    try:
        technicians = TechnicianModel.query.filter_by(active=True).all()
        return jsonify([{
            'id': tech.id,
            'name': tech.name,
            'email': tech.email,
            'phone': tech.phone,
            'skills': tech.skills.split(',') if tech.skills else [],
            'workHours': f"{tech.work_start_time.strftime('%H:%M')} - {tech.work_end_time.strftime('%H:%M')}",
            'maxCallsPerDay': tech.max_calls_per_day
        } for tech in technicians])
    except Exception as e:
        logger.error(f"Error fetching technicians: {e}")
        return jsonify({'error': 'Failed to fetch technicians'}), 500


@app.route('/api/technicians', methods=['POST'])
def create_technician():
    """Create a new technician"""
    try:
        data = request.json
        
        # Generate ID if not provided
        if 'id' not in data:
            data['id'] = f"T{datetime.utcnow().strftime('%Y%m%d%H%M%S')}"
        
        tech = TechnicianModel(
            id=data['id'],
            name=data['name'],
            email=data.get('email'),
            phone=data.get('phone'),
            skills=','.join(data.get('skills', [])),
            home_address=data.get('homeAddress'),
            home_latitude=data.get('homeLatitude'),
            home_longitude=data.get('homeLongitude'),
            work_start_time=datetime.strptime(data.get('workStartTime', '08:00'), '%H:%M').time(),
            work_end_time=datetime.strptime(data.get('workEndTime', '17:00'), '%H:%M').time(),
            max_calls_per_day=data.get('maxCallsPerDay', 8)
        )
        
        db.session.add(tech)
        db.session.commit()
        
        return jsonify({
            'id': tech.id,
            'message': 'Technician created successfully'
        }), 201
        
    except Exception as e:
        logger.error(f"Error creating technician: {e}")
        db.session.rollback()
        return jsonify({'error': 'Failed to create technician'}), 500


@app.route('/api/optimize', methods=['POST'])
def optimize_routes():
    """Optimize routes for a given date"""
    try:
        data = request.json
        target_date = datetime.fromisoformat(data['date']).date() if 'date' in data else datetime.utcnow().date()
        
        # Fetch service calls for the date
        db_calls = ServiceCallModel.query.filter_by(
            scheduled_date=target_date,
            status='pending'
        ).all()
        
        # Fetch active technicians
        db_technicians = TechnicianModel.query.filter_by(active=True).all()
        
        if not db_calls:
            return jsonify({
                'routes': [],
                'unassigned': [],
                'warnings': ['No pending service calls for the selected date']
            })
        
        # Convert to optimization objects
        service_calls = []
        for call in db_calls:
            # Combine date and time for datetime objects
            start_datetime = datetime.combine(call.scheduled_date, call.time_window_start)
            end_datetime = datetime.combine(call.scheduled_date, call.time_window_end)
            
            service_calls.append(ServiceCall(
                id=call.id,
                customer_name=call.customer_name,
                location=Location(call.address, call.latitude, call.longitude),
                time_window=TimeWindow(start_datetime, end_datetime),
                duration_minutes=call.duration_minutes,
                priority=Priority[call.priority],
                required_skills=[SkillType(s) for s in call.required_skills.split(',') if s]
            ))
        
        technicians = []
        for tech in db_technicians:
            # Combine date and time for datetime objects
            start_datetime = datetime.combine(target_date, tech.work_start_time)
            end_datetime = datetime.combine(target_date, tech.work_end_time)
            
            technicians.append(Technician(
                id=tech.id,
                name=tech.name,
                skills=[SkillType(s) for s in tech.skills.split(',') if s],
                home_location=Location(
                    tech.home_address or "Office",
                    tech.home_latitude or 40.7580,
                    tech.home_longitude or -73.9855
                ),
                work_hours=TimeWindow(start_datetime, end_datetime),
                max_calls_per_day=tech.max_calls_per_day
            ))
        
        # Run optimization
        result = optimizer.optimize(service_calls, technicians)
        
        # Format response
        formatted_routes = []
        for route in result['routes']:
            formatted_routes.append({
                'technician': {
                    'id': route['technician'].id,
                    'name': route['technician'].name
                },
                'calls': [{
                    'id': stop['call'].id,
                    'customerName': stop['call'].customer_name,
                    'address': stop['call'].location.address,
                    'arrivalTime': stop['arrival_time'],
                    'departureTime': stop['departure_time']
                } for stop in route['calls']],
                'totalDistance': round(route['total_distance'], 2),
                'utilization': round(route['utilization'] * 100, 1)
            })
        
        formatted_unassigned = [{
            'id': call.id,
            'customerName': call.customer_name,
            'address': call.location.address,
            'reason': 'No available technician with required skills'
        } for call in result['unassigned']]
        
        return jsonify({
            'routes': formatted_routes,
            'unassigned': formatted_unassigned,
            'statistics': result['statistics'],
            'warnings': result['warnings']
        })
        
    except Exception as e:
        logger.error(f"Error optimizing routes: {e}")
        return jsonify({'error': f'Failed to optimize routes: {str(e)}'}), 500


@app.route('/api/save-schedule', methods=['POST'])
def save_schedule():
    """Save the optimized schedule"""
    try:
        data = request.json
        routes = data.get('routes', [])
        
        # Update service calls with assignments
        for route in routes:
            technician_id = route['technician']['id']
            for call in route['calls']:
                db_call = ServiceCallModel.query.get(call['id'])
                if db_call:
                    db_call.assigned_technician_id = technician_id
                    db_call.status = 'scheduled'
        
        db.session.commit()
        
        return jsonify({'message': 'Schedule saved successfully'}), 200
        
    except Exception as e:
        logger.error(f"Error saving schedule: {e}")
        db.session.rollback()
        return jsonify({'error': 'Failed to save schedule'}), 500


# Initialize database
@app.before_request
def create_tables():
    db.create_all()
    logger.info("Database tables created")


# Error handlers
@app.errorhandler(404)
def not_found(error):
    return jsonify({'error': 'Not found'}), 404


@app.errorhandler(500)
def internal_error(error):
    return jsonify({'error': 'Internal server error'}), 500


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5001)
