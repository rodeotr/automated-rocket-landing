
#include <box2d\box2d.h>

class Rocket {
public:
	Rocket(float width_, float height_, float density_, b2Vec2 position_, b2Vec2 velocity_, b2World& World_) {
		width = width_;
		height = height_;
		density = density_;
		position = position_;
		velocity = velocity_;
		World = &World_;
	};
	b2Body* create_rocket() {
		b2BodyDef myBodyDef;
		myBodyDef.type = b2_dynamicBody;
		b2Vec2 vertices[4];
		//vertices[2].y = height;
		vertices[0].Set(0, 0);
		vertices[1].Set(width, 0);
		vertices[2].Set(width, height);
		vertices[3].Set(0, height);
		b2PolygonShape polygonShape;
		polygonShape.Set(vertices, 4);
		b2FixtureDef myFixtureDef;
		myFixtureDef.density = 150.0f;
		myFixtureDef.friction = 0.7f;
		myFixtureDef.restitution = 1.0f;
		myFixtureDef.shape = &polygonShape;

		myBodyDef.position.Set(-30, -50);
		float velocity = 70;
		//myBodyDef.linearVelocity.Set(sinf(0.3)* velocity,cosf(0.3)* velocity);
		myBodyDef.angle = -0.3;
		b2Body* Body = World->CreateBody(&myBodyDef);
		b2PolygonShape leftLeg;
		leftLeg.SetAsBox(0.3f, 6.0f, b2Vec2(-5.0f, 50.0f), 1.0f);
		b2PolygonShape rightLeg;
		rightLeg.SetAsBox(0.3f, 6.0f, b2Vec2(8.0f, 50.0f), -1.0f);

		b2RevoluteJointDef jd;
		jd.bodyA = Body;
		b2MassData massdata;


		Body->CreateFixture(&myFixtureDef);
		Body->CreateFixture(&leftLeg, 10);
		Body->CreateFixture(&rightLeg, 10);
		b2Fixture* fixture = Body->GetFixtureList()->GetNext()->GetNext();

		Body->GetMassData(&massdata);

		massdata.center = b2Vec2(1.5, 40.0f);

		Body->SetLinearVelocity(b2Vec2(sinf(0.3) * velocity, cosf(0.3) * velocity));

		return Body;
	}
private:
	float width;
	float height;
	float density;
	b2Vec2 position;
	b2Vec2 velocity;
	b2World* World;
};