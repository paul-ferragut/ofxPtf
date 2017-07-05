/*
 *  ParallelTransportFrames.cpp
 *
 *  Copyright (c) 2012, Neil Mendoza, http://www.neilmendoza.com
 *  All rights reserved. 
 *  
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions are met: 
 *  
 *  * Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer. 
 *  * Redistributions in binary form must reproduce the above copyright 
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the distribution. 
 *  * Neither the name of Neil Mendoza nor the names of its contributors may be used 
 *    to endorse or promote products derived from this software without 
 *    specific prior written permission. 
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 *  POSSIBILITY OF SUCH DAMAGE. 
 *
 */
#include "ParallelTransportFrames.h"

namespace itg
{
    ParallelTransportFrames::ParallelTransportFrames() :
        maxPoints(4), maxFrames(numeric_limits<unsigned>::max())
    {
        
    }
    
    bool ParallelTransportFrames::addPoint(const ofVec3f& point)
    {
        points.push_back(point);
        while (points.size() > maxPoints) points.pop_front();
        if (points.size() == 3) firstFrame();
        else if (points.size() > 3)
        {
            nextFrame();
            return true;
        }
        return false;
    }
                          
    void ParallelTransportFrames::firstFrame()
    {
        ofVec3f t = ( points[1] - points[0] ).normalize();

        ofVec3f n = t.crossed( points[2] - points[0] ).normalize();
        if( n.length() == 0.0f )
        {
            int i = fabs( t[0] ) < fabs( t[1] ) ? 0 : 1;
            if( fabs( t[2] ) < fabs( t[i] ) ) i = 2;

            ofVec3f v;
            v[i] = 1.f;
            n = t.crossed( v ).normalize();
        }

        ofVec3f b = t.crossed( n );

        ofMatrix4x4 m(t[0], t[1], t[2], 0.0,
                      b[0], b[1], b[2], 0.0,
                      n[0], n[1], n[2], 0.0,
                      points[0][0], points[0][1], points[0][2], 1.f); 
            
        frames.push_back(m);
        
        prevTangent = t;
        startNormal = n;
    }
    
    void ParallelTransportFrames::nextFrame()
    {
        curTangent = points.back() - points[points.size() - 2];
        ofVec3f a;	// Rotation axis.
        float r = 0;						// Rotation angle.
        
        if( ( prevTangent.lengthSquared() != 0.0 ) && ( curTangent.lengthSquared() != 0.0 ) )
        {
            curTangent.normalize();
            float dot = prevTangent.dot( curTangent ); 
            
            if( dot > 1.f ) dot = 1.f; 
            else if( dot < -1.0 ) dot = -1.0;
            
            r = acos( dot );
            a = prevTangent.crossed( curTangent );
        }
        
        if( ( a.length() != 0.0 ) && ( r != 0.0 ) )
        {
            ofMatrix4x4 R;
            R.makeRotationMatrix(RAD_TO_DEG * r, a);		
            ofMatrix4x4 Tj;
            Tj.makeTranslationMatrix( points.back() );
            ofMatrix4x4 Ti;
            Ti.makeTranslationMatrix( -points[points.size() - 2] );
            
            frames.push_back(frames.back() * Ti * R * Tj);
        }
        else
        {
            ofMatrix4x4 Tr;
            Tr.makeTranslationMatrix( points.back() - points[points.size() - 2] );
            
            frames.push_back(frames.back() * Tr);
        }
        prevTangent = curTangent;
        while (frames.size() > maxFrames) frames.pop_front();
    }
    
    ofMatrix4x4 ParallelTransportFrames::normalMatrix() const
    {
        ofMatrix4x4 normalMatrix = ofMatrix4x4::getTransposedOf(const_cast<ofMatrix4x4&>(frames.back()).getInverse());
        return ofMatrix4x4(normalMatrix(0, 0), normalMatrix(0, 1), normalMatrix(0, 2), 0.f,
                           normalMatrix(1, 0), normalMatrix(1, 1), normalMatrix(1, 2), 0.f,
                           normalMatrix(2, 0), normalMatrix(2, 1), normalMatrix(2, 2), 0.f,
                           0.f,                0.f,                0.f,                1.f);
    }
    
    ofVec3f ParallelTransportFrames::calcCurrentNormal() const
    {
        return getStartNormal() * normalMatrix();
    }
    
    void ParallelTransportFrames::debugDraw(float axisSize)
    {
        ofSetColor(0, 255, 255);
        ofNoFill();
        for (int i = 0; i < frames.size(); ++i)
        {
            ofPushMatrix();
            ofMultMatrix(frames[i]);
            ofRotate(90, 0, 1, 0);
            ofCircle(0, 0, axisSize * 2.f);
            ofDrawAxis(axisSize);
            ofPopMatrix();
        }
    }
    
    void ParallelTransportFrames::clear()
    {
        points.clear();
        frames.clear();
    }

	/*
	void ParallelTransportFrames::setupPtfPoints(vector<ofVec3f>ptfPoints,float resolutionRing,float widthRing, float heightRing) {
		for (int i = 0;i < ptfPoints.size();i++) {
			addPoint(ptfPoints[i]);
		}

		//SET CURVED RING SHAPE
		ofVboMesh mesh;
		for (int i = 0; i < resolutionRing; i++)
		{
			mesh.addVertex(ofVec3f(0.f, widthRing * cos(TWO_PI * i / (float)resolutionRing), heightRing * sin(TWO_PI * i / (float)resolutionRing)));
		}
		//

		//APPLY PTF TRANSFORM
		vector<ofVboMesh>rings;
		for (int i = 0; i < framesSize(); i++)
		{
			ofMatrix4x4 mat4 = frameAt(i);
			ofVboMesh tempMesh = mesh;
			for (int j = 0;j < tempMesh.getNumVertices(); j++) {
				tempMesh.setVertex(j, mesh.getVertex(j)*mat4);
			}
			rings.push_back(tempMesh);
		}
		//


		//POPULATE OUR MESH
		meshptf.clear();
		meshptf.setMode(OF_PRIMITIVE_TRIANGLES);
		int index = 0;
		for (int i = 0;i < rings.size();i++) {
			for (int j = 0;j < rings[i].getNumVertices();j++) {
				meshptf.addVertex(rings[i].getVertex(j));
				//meshptf.addColor(ofFloatColor(fabs(cos((index*0.1))), fabs(cos((index*0.1) + 1)), fabs(cos((index*0.1) + 2))));
				index++;
			}
		}

		//ADD TRIANGLES TO FORM A MESH
		for (int i = 0;i < meshptf.getNumVertices() - (resolutionRing);i++) {

			if (i != meshptf.getNumVertices() - (resolutionRing + 1)) {
				meshptf.addTriangle(i, i + 1, i + (resolutionRing));
				meshptf.addTriangle(i + (resolutionRing), i + (resolutionRing + 1), i + 1);
			}
			//last
			else {
				meshptf.addTriangle(i, i + 1, i + resolutionRing);
			}
			//first
			if (i == 0) {
				meshptf.addTriangle(i, resolutionRing - 1, resolutionRing);
			}
		}

	}
	*/




	void ParallelTransportFrames::setupPtfPoints(vector<ofVec3f>ptfPoints, float resolutionRing, float radiusRingStart, float radiusRingEnd) {
		for (int i = 0;i < ptfPoints.size();i++) {
			addPoint(ptfPoints[i]);
		}

		//SET CURVED RING SHAPE
		/*
		ofVboMesh mesh;
		for (int i = 0; i < resolutionRing; i++)
		{
			mesh.addVertex(ofVec3f(0.f, widthRing * cos(TWO_PI * i / (float)resolutionRing), heightRing * sin(TWO_PI * i / (float)resolutionRing)));
		}
		*/
		//

		//APPLY PTF TRANSFORM
		vector<ofVboMesh>rings;
		for (int i = 0; i < framesSize(); i++)
		{

			float transitionRadius = radiusRingEnd;
			if (i < (framesSize() / 2)) {
				
				transitionRadius = ofLerp(radiusRingEnd,radiusRingStart , ofMap(i, 0,framesSize() / 2 ,0 ,1 , true));
			
			}
			else {
				
				transitionRadius = ofLerp(radiusRingStart, radiusRingEnd, ofMap(i, (framesSize() / 2), framesSize(), 0, 1,true));
			
			}

			ofMatrix4x4 mat4 = frameAt(i);

			ofVboMesh mesh;
			for (int i = 0; i < resolutionRing; i++)
			{
				mesh.addVertex(ofVec3f(0.f, transitionRadius * cos(TWO_PI * i / (float)resolutionRing), transitionRadius * sin(TWO_PI * i / (float)resolutionRing)));
			}

			ofVboMesh tempMesh = mesh;
			for (int j = 0;j < tempMesh.getNumVertices(); j++) {
				tempMesh.setVertex(j, mesh.getVertex(j)*mat4);
			}
			rings.push_back(tempMesh);
		}
		//


		//POPULATE OUR MESH
		meshptf.clear();
		meshptf.setMode(OF_PRIMITIVE_TRIANGLES);
		int index = 0;
		for (int i = 0;i < rings.size();i++) {
			for (int j = 0;j < rings[i].getNumVertices();j++) {
				meshptf.addVertex(rings[i].getVertex(j));

				meshptf.addColor(ofFloatColor(ofMap(i,0, rings.size(),1.0f,0.0f), ofMap(i, 0, rings.size(), 1.0f, 0.0f), ofMap(i, 0, rings.size(), 1.0f, 0.0f)));
				index++;
			}
		}

		//ADD TRIANGLES TO FORM A MESH
		for (int i = 0;i < meshptf.getNumVertices() - (resolutionRing);i++) {

			if (i != meshptf.getNumVertices() - (resolutionRing + 1)) {
				meshptf.addTriangle(i, i + 1, i + (resolutionRing));
				meshptf.addTriangle(i + (resolutionRing), i + (resolutionRing + 1), i + 1);
			}
			//last
			else {
				meshptf.addTriangle(i, i + 1, i + resolutionRing);
			}
			//first
			if (i == 0) {
				meshptf.addTriangle(i, resolutionRing - 1, resolutionRing);
			}
		}

	}






	
	ofVboMesh ParallelTransportFrames::getMesh() {
	
		return meshptf;

	}
}