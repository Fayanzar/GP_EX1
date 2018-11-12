#include "MassSpringSystemSimulator.h"

MassSpringSystemSimulator::MassSpringSystemSimulator()
{
	m_fMass = 10;
	m_fStiffness = 40;
	m_fDamping = 0;
	m_iIntegrator = 0;
}

void MassSpringSystemSimulator::setDampingFactor(float damping) {
	m_fDamping = damping;
}

void MassSpringSystemSimulator::setMass(float mass) {
	m_fMass = mass;
}

void MassSpringSystemSimulator::setStiffness(float stiffness) {
	m_fStiffness = stiffness;
}

int MassSpringSystemSimulator::addMassPoint(Vec3 position, Vec3 velocity, bool isFixed) {
	m_mMassPoints.push_back(MassPoint(position, velocity, isFixed));
	return (m_mMassPoints.size() - 1);
}

void MassSpringSystemSimulator::addSpring(int masspoint1, int masspoint2, float initialLength) {
	m_sSprings.push_back(Spring(masspoint1, masspoint2, initialLength));
}

int MassSpringSystemSimulator::getNumberOfMassPoints() {
	return m_mMassPoints.size();
}

int MassSpringSystemSimulator::getNumberOfSprings() {
	return m_sSprings.size();
}

Vec3 MassSpringSystemSimulator::getPositionOfMassPoint(int index) {
	return m_mMassPoints.at(index).position;
}

Vec3 MassSpringSystemSimulator::getVelocityOfMassPoint(int index) {
	return m_mMassPoints.at(index).velocity;
}

void MassSpringSystemSimulator::applyExternalForce(Vec3 force) {
	if (m_iTestCase == 2) {
		m_externalForce = force;
	}
}

const char * MassSpringSystemSimulator::getTestCasesStr() {
	return "Test1,Setup1,Setup2";
}

void MassSpringSystemSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
}

void MassSpringSystemSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void MassSpringSystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	DUC->setUpLighting(Vec3(), 0.4*Vec3(1, 1, 1), 100, 0.6*Vec3(0.97, 0.86, 1));
	for each (Spring spring in m_sSprings)
	{
		DUC->beginLine();
		DUC->drawLine(m_mMassPoints.at(spring.masspoint1).position, Vec3(1, 1, 1), m_mMassPoints.at(spring.masspoint2).position, Vec3(1, 1, 1));
		DUC->endLine();
	}
	for each (MassPoint massPoint in m_mMassPoints)
	{
		DUC->drawSphere(massPoint.position, Vec3(0.01, 0.01, 0.01));
	}
}

void MassSpringSystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void MassSpringSystemSimulator::externalForcesCalculations(float timeElapsed)
{
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	Point2D mouseDiff;
	mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	if (mouseDiff.x != 0 || mouseDiff.y != 0)
	{
		Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
		worldViewInv = worldViewInv.inverse();
		Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
		Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
		// find a proper scale!
		float inputScale = 0.1f;
		inputWorld = inputWorld * inputScale;
		applyExternalForce(inputWorld);
	}
	else {
		applyExternalForce(Vec3(0, 0, 0));
	}
}

void MassSpringSystemSimulator::notifyMethodChanged(int method) {
	if (m_iTestCase > 0) {
		notifyCaseChanged(m_iTestCase);
	}
	setIntegrator(method);
}

void MassSpringSystemSimulator::notifyCaseChanged(int testCase) {
	m_iTestCase = testCase;
	m_sSprings.clear();
	m_mMassPoints.clear();
	switch (m_iTestCase) {
	case 0: {
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);
		Euler(0.1);

		m_sSprings.clear();
		m_mMassPoints.clear();

		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);

		midpoint(0.1);
		break; 
	}
	case 1: {
		addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);
		addSpring(0, 1, 1);
		break;
	}
	case 2: {
		addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.5, 0, 0), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.5, 0, 0.5), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0, 0, 0.5), Vec3(0, 0, 0), false);

		addMassPoint(Vec3(0, 0.5, 0), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.5, 0.5, 0), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.5, 0.5, 0.5), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0, 0.5, 0.5), Vec3(0, 0, 0), false);

		addSpring(0, 1, 0.5);
		addSpring(2, 1, 0.5);
		addSpring(2, 3, 0.5);
		addSpring(0, 3, 0.5);

		addSpring(4, 5, 0.5);
		addSpring(5, 6, 0.5);
		addSpring(6, 7, 0.5);
		addSpring(4, 7, 0.5);

		addSpring(4, 0, 0.5);
		addSpring(5, 1, 0.5);
		addSpring(6, 2, 0.5);
		addSpring(3, 7, 0.5);

		addSpring(4, 0, 0.5);
		addSpring(5, 1, 0.5);
		addSpring(6, 2, 0.5);
		addSpring(3, 7, 0.5);

		addSpring(0, 2, 0.707);
		addSpring(5, 7, 0.707);
		addSpring(0, 5, 0.707);
		addSpring(1, 6, 0.707);
		addSpring(2, 7, 0.707);
		addSpring(3, 4, 0.707);

		addSpring(1, 3, 0.707);
		addSpring(4, 6, 0.707);
		addSpring(1, 4, 0.707);
		addSpring(2, 5, 0.707);
		addSpring(3, 6, 0.707);
		addSpring(0, 7, 0.707);

		addMassPoint(Vec3(0, 1, 0), Vec3(0, 0, 0), true);
		addMassPoint(Vec3(0.5, 0.5, -0.5), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0.5, 0.5, 0.5), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(-0.5, 0.5, 0.5), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(-0.5, 0.5, -0.5), Vec3(0, 0, 0), false);
		addMassPoint(Vec3(0, 0, 0), Vec3(0, 0, 0), false);

		addSpring(8, 9, 0.707);
		addSpring(8, 10, 0.707);
		addSpring(8, 11, 0.707);
		addSpring(8, 12, 0.707);

		addSpring(13, 9, 0.707);
		addSpring(13, 10, 0.707);
		addSpring(13, 11, 0.707);
		addSpring(13, 12, 0.707);

		addSpring(9, 10, 1);
		addSpring(10, 11, 1);
		addSpring(12, 11, 1);
		addSpring(9, 12, 1);
		break;
	}
	}
}

void MassSpringSystemSimulator::simulateTimestep(float timestep) {
	if (m_iTestCase > 0) {
		switch (m_iIntegrator) {
		case 0: {
			Euler(timestep);
			break;
		}
		case 1: {
			midpoint(timestep);
			break;
		}
		}
	}
}

void MassSpringSystemSimulator::Euler(float timestep) {
	for (int i = 0; i < m_mMassPoints.size(); i++) {
		if (!m_mMassPoints.at(i).isFixed) {
			Vec3 forces = calculateForces(i);
			MassPoint* massPoint = &m_mMassPoints.at(i);
			Vec3 gravity = m_iTestCase == 2 ? m_fMass * Vec3(0, 0.1, 0) : Vec3(0, 0, 0);
			Vec3 new_pos = massPoint->position + timestep * massPoint->velocity;
			Vec3 new_vel = massPoint->velocity + timestep * (forces - gravity - massPoint->velocity * m_fDamping) / m_fMass;

			massPoint->position = new_pos;
			if (massPoint->position.Y < -1 && m_iTestCase == 2) 
				massPoint->position.Y = -1;
			
			massPoint->velocity = new_vel;
			if (m_iTestCase == 0) {
				cout << "Euler method. Position of " << i << " point: " << new_pos << endl;
				cout << "Euler method. Velocity of " << i << " point: " << new_vel << endl;
			}
		}
	}
}

void MassSpringSystemSimulator::midpoint(float timestep) {
	for (int i = 0; i < m_mMassPoints.size(); i++) {
		if (!m_mMassPoints.at(i).isFixed) {
			Vec3 forces = calculateForces(i);
			MassPoint* massPoint = &m_mMassPoints.at(i);
			Vec3 gravity = m_iTestCase == 2 ? m_fMass * Vec3(0, 0.1, 0) : Vec3(0, 0, 0);
			Vec3 y_x2 = massPoint->velocity + (forces - gravity - massPoint->velocity * m_fDamping) / m_fMass * timestep / 2.;
			Vec3 y1_x2 = massPoint->velocity + timestep * (forces - gravity - y_x2 * m_fDamping) / m_fMass;
			Vec3 y1_x1 = massPoint->position + timestep * y1_x2;

			massPoint->position = y1_x1;
			if (m_iTestCase == 2 && massPoint->position.Y < -1)
				massPoint->position.Y = -1;

			massPoint->velocity = y1_x2;

			if (m_iTestCase == 0) {
				cout << "Midpoint method. Position of " << i << " point: " << y1_x1 << endl;
				cout << "Midpoint method. Velocity of " << i << " point: " << y1_x2 << endl;
			}
		}
	}
}

Vec3 MassSpringSystemSimulator::calculateForces(int index) {
	// this thing calculates forces for the point x_i, use it every time you need it
	Vec3 force = Vec3(0, 0, 0);
	for each (Spring spring in m_sSprings)
	{
		if (spring.masspoint1 == index) {
			float length = m_mMassPoints.at(spring.masspoint1).position.squaredDistanceTo(m_mMassPoints.at(spring.masspoint2).position);
			length = sqrt(length);
			force -= m_fStiffness * (length - spring.initialLength) / length * (m_mMassPoints.at(spring.masspoint1).position - m_mMassPoints.at(spring.masspoint2).position);
		}
		else if (spring.masspoint2 == index) {
			float length = m_mMassPoints.at(spring.masspoint1).position.squaredDistanceTo(m_mMassPoints.at(spring.masspoint2).position);
			length = sqrt(length);
			force += m_fStiffness * (length - spring.initialLength) / length * (m_mMassPoints.at(spring.masspoint1).position - m_mMassPoints.at(spring.masspoint2).position);
		}
	}
	force += m_externalForce;

	return force;
}