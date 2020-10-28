#include <SFML\Graphics.hpp>
#include <box2d\box2d.h>
#include <iostream>
#include <math.h>
#include "SFMLDebugDraw.h"
#include "PID_Controller.h"
#include "objects.cpp"


b2Body* rocket_body;
float angle_ = -6.5 * b2_pi / 180;
float power = 850000;
bool contact_ = false;
bool stop = false;

class myListener : public b2ContactListener {
	
	void BeginContact(b2Contact* contact);
	void EndContact(b2Contact* contact) {}
	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold){}
	void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse){}
};
void myListener::BeginContact(b2Contact* contact) {
	b2Vec2 velocity = (rocket_body)->GetLinearVelocity();

	power = 0;
	stop = true;
	contact_ = true;
	//std::cout << "Contact!\n";
	//static_cast<b2Body*>(my_body)->getTransform();
	
}

/** We need this to easily convert between pixel and real-world coordinates*/
static const float SCALE = 32.0f;
const float falcon9_height = 50.0f;
const float falcon9_width = 3.0f;
std::vector<b2Body*> fuel_particles;

/** Create the base for the boxes to land */
void CreateGround(b2World& World, float X, float Y);
void createCircle(b2World& World, b2Vec2 position, float angle);
/** Create the boxes */
//b2Body* CreateShape_(b2World& World);

int main()
{
	/** Prepare the window */
	sf::RenderWindow Window(sf::VideoMode(800, 600, 32), "Test");
	Window.setFramerateLimit(60);
	
	// scale 50 -> set origin 75.0f, 400.0f
	// scale 30 -> set origin (11.0f, 190.0f)
	

	sf::Event event;


	/** Prepare the world */
	b2Vec2 Gravity(0.f,9.8f);
	b2World World(Gravity);
	myListener listener;
	World.SetContactListener(&listener);
	CreateGround(World, 400.f, 500.f);

	sf::View view(sf::FloatRect(0.0f,0.f,800.f,600.f));
	view.zoom(10);
	Window.setView(view);

	SFMLDebugDraw debugDraw(Window);
	//SFMLDebugDraw debugDraw_1(Window);
	//World.SetDebugDraw(&debugDraw_1);
	//debugDraw_1.SetFlags(b2Draw::e_shapeBit);
	World.SetDebugDraw(&debugDraw);
	debugDraw.SetFlags(17);
	
	PID_Controller pid_thrust = PID_Controller(20000, 0, 0);
	float result = pid_thrust.pid_execute(10, 2);
	PID_Controller pid_thrust_angle = PID_Controller(15, 0, 0);
	std::cout << "pid value is " << result <<  "\n";

	/** Prepare textures */
	sf::Texture GroundTexture;
	sf::Texture BoxTexture;
	GroundTexture.loadFromFile("sky_small.png");
	BoxTexture.loadFromFile("falcon9_icon.png");
	
	bool initialized = false;
	
	int counter = 0;
	b2Vec2 vect_;
	vect_.x = 0.0f;
	vect_.y = -3.8;
	//my_body = CreateBox(World, 10, 10);
	//my_body = CreateShape_(World);
	Rocket rocket = Rocket(falcon9_width, falcon9_height, 0, b2Vec2(0, 0), b2Vec2(0, 0), World);
	rocket_body = rocket.create_rocket();
	//createCircle(World,b2Vec2(0,10));
	//my_body = createDrop(World);
	/*sf::CircleShape circle(15);
	circle.setFillColor(sf::Color::Red);
	circle.setPosition(400, 100);*/
	
	sf::Sprite Sprite;
	Sprite.setTexture(BoxTexture);
	b2Vec2 thrust_point;
	
	while (Window.isOpen())
	{
		while (Window.pollEvent(event)) {
			if (event.type == sf::Event::Closed) {
				Window.close();
			}
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Up) {
				b2Vec2 end_point;
				
			/*	b2Fixture* fix = my_body->GetFixtureList()->GetNext();
				b2Shape* polygonShape = fix->GetShape();*/
				//b2Vec2* points = polygonShape->m_vertices;
				
				for (b2Fixture* f = rocket_body->GetFixtureList(); f; f = f->GetNext())
				{
					b2PolygonShape* polygonShape = (b2PolygonShape*)f->GetShape();
					b2Vec2* my_vec = polygonShape->m_vertices;
					thrust_point.x = (my_vec[1].x + my_vec[2].x) / 2;
					thrust_point.y = (my_vec[1].y + my_vec[2].y) / 2;				
				}



				
				vect_.x = sinf(angle_ + rocket_body->GetAngle())*power;
				vect_.y = -1*cosf(angle_ + rocket_body->GetAngle()) * power;
				//std::cout << vect_.x << ", " << vect_.y << "\n";
				//std::cout << vect_.x << "\n";
				(rocket_body)->ApplyForce(vect_, rocket_body->GetWorldPoint(thrust_point), false);
				//my_body->SetAngularVelocity(0.1);
				//my_body->SetTransform(my_body->GetPosition(), angle_);
				//my_body->GetWorldCenter
			}
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Right) {
				angle_ -= 0.001;
			}
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Left)
			{
				angle_ += 0.001;
			}
			if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Space)
			{
				stop = true;
			}
		}
		
		for (b2Fixture* f = rocket_body->GetFixtureList(); f; f = f->GetNext())
		{
			b2PolygonShape* polygonShape = (b2PolygonShape*)f->GetShape();
			b2Vec2* my_vec = polygonShape->m_vertices;
			thrust_point.x = (my_vec[1].x + my_vec[2].x) / 2;
			thrust_point.y = (my_vec[1].y + my_vec[2].y) / 2;

		}
		if (rocket_body->GetLinearVelocity().y < 15 && !contact_) {
			//power = 200000;
			power = 0;
			angle_ = -10 * b2_pi / 180;

		}
		if (rocket_body->GetLinearVelocity().y < 1) {
			contact_ = true;
			power = 0;
		}
		
		//if ((my_body->GetAngle() * 180 / b2_pi) < -30) {
		//	//std::cout << "power increased\n";
		//	power = 1000000;
		//}
		if (stop) {
			continue;
		}
		power = 1 * pid_thrust.pid_execute(0, rocket_body->GetPosition().y);
		std::cout << power << "\n";
		vect_.x = sinf(angle_ + rocket_body->GetAngle()) * power;
		vect_.y = -1 * cosf(angle_ + rocket_body->GetAngle()) * power;
		if (!contact_) {
			(rocket_body)->ApplyForce(vect_, rocket_body->GetWorldPoint(thrust_point), false);
			if (counter == 5) {
				if (fuel_particles.size() > 50) {
					b2Vec2 velocity = (rocket_body)->GetLinearVelocity();
					if (velocity.Length() == 0) {
						stop = true;
						std::cout << "stopped\n";
					}
					//fuel_particles.pop_back();
					//int a = World.GetBodyList().size();
					//World.DestroyBody(fuel_particles[0]);
				}
				//std::cout << my_body->GetLinearVelocity().x << "\n";
				createCircle(World, rocket_body->GetWorldPoint(thrust_point), angle_ + rocket_body->GetAngle());
				counter = 0;
			}
		}
		
		
		
		Window.clear(sf::Color(0,0,0));	
		World.Step(1 / 60.f, 8, 3);

		b2Vec2 end_point;
		int BodyCount = 0;
		for (b2Body* BodyIterator = World.GetBodyList(); BodyIterator != 0; BodyIterator = BodyIterator->GetNext())
		{
			
			if (BodyIterator->GetType() == b2_dynamicBody)
			{
				
				Sprite.setOrigin(SCALE* BodyIterator->GetPosition().x, SCALE* BodyIterator->GetPosition().y);
				Sprite.setScale(3/SCALE, 4/SCALE);
				Sprite.setPosition(SCALE * BodyIterator->GetPosition().x + 250, 3.55*SCALE * BodyIterator->GetPosition().y);
				Sprite.setRotation(BodyIterator->GetAngle() * 180 / b2_pi);
				//circle.setPosition(SCALE* BodyIterator->GetWorldCenter().x, SCALE * BodyIterator->GetWorldCenter().y);
				//Sprite.setRotation(BodyIterator->GetAngle() * 180 / b2_pi);
				//circle.setPosition(200, SCALE*thrust_point.y);
				//Window.draw(circle);
				//Window.draw(Sprite);
				//BodyIterator->SetTransform(BodyIterator->GetWorldCenter(),angle_);
				++BodyCount;
				//std::cout << my_body->GetAngle()*180/b2_pi << "\n";
				//std::cout << SCALE * BodyIterator->GetWorldCenter().x << ", " << SCALE * BodyIterator->GetWorldCenter().y << "\n";
				//std::cout << SCALE * BodyIterator->GetPosition().x << ", " << SCALE * BodyIterator->GetPosition().y << "\n\n";
				//std::cout << SCALE * BodyIterator->GetPosition().y << "\n";
				
				/*end_point.x = BodyIterator->GetPosition().x * SCALE - 2;
				end_point.y = BodyIterator->GetPosition().y * SCALE  + 70;*/
				//(my_body)->ApplyForce(vect_,b2Vec2(SCALE* BodyIterator->GetPosition().x-2, SCALE* BodyIterator->GetPosition().y+300),false);
				
				/*circle.setPosition(sf::Vector2f(end_point.x, end_point.x));
				Window.draw(circle);*/
			}
			else
			{
				//std::cout << BodyIterator->GetPosition().x * SCALE << "  " << BodyIterator->GetPosition().y * SCALE<<" \n";
				sf::Sprite GroundSprite;
				GroundSprite.setTexture(GroundTexture);
				GroundSprite.setOrigin(375.0f, 10.f);
				GroundSprite.setScale(3 / SCALE, 3 / SCALE);
				GroundSprite.setPosition(BodyIterator->GetPosition().x * SCALE, BodyIterator->GetPosition().y * SCALE);
				//GroundSprite.setRotation(180 / b2_pi * BodyIterator->GetAngle());
				//GroundSprite.getGlobalBounds().intersects()
				
				//Window.draw(GroundSprite);
			}
		}

		//std::cout << my_body->GetWorldPoint(thrust_point).x  << ", " << my_body->GetWorldPoint(thrust_point).y << "\n";
		World.DebugDraw();
		Window.display();
		//stop = true;
	}

	return 0;
}




void CreateGround(b2World& World, float X, float Y)
{
	Y = 500;
	b2BodyDef BodyDef;
	BodyDef.position.y = 80.0f;
	

	b2Body* Body = World.CreateBody(&BodyDef);
	
	b2PolygonShape Shape;
	Shape.SetAsBox((220.0f / 2), (0.5f / 2));
	
	b2FixtureDef FixtureDef;
	FixtureDef.density = 1.0f;
	FixtureDef.friction = 0.7;
	FixtureDef.shape = &Shape;
	Body->CreateFixture(&FixtureDef); 

}

void createCircle(b2World& World, b2Vec2 position, float angle) {
	b2BodyDef bd;
	bd.type = b2_kinematicBody;
	bd.position = position;
	
	b2Body* m_bomb = World.CreateBody(&bd);
	m_bomb->SetLinearVelocity(b2Vec2(50*sinf(angle), 50 * cosf(angle)));
	

	b2CircleShape circle;
	circle.m_radius = 1.0f;

	b2FixtureDef fd;
	b2Filter my_filter;
	my_filter.categoryBits = 0;
	my_filter.maskBits = 0;
	fd.filter = my_filter;
	fd.isSensor = true;
	fd.shape = &circle;
	fd.density = 0.0f;
	fd.restitution = 0.0f;

	m_bomb->CreateFixture(&fd);
	fuel_particles.push_back(m_bomb);
}

