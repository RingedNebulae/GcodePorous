#include "Gcode.h"


Gcode::Gcode()
{
}


Gcode::~Gcode()
{
}

bool Gcode::readInCuraGcode(string fileName)
{
	string inputbuffer;//为了读入文件的便于格式转化
	vector<string> inputGcode;

	ifstream openGcode(fileName);
	if (!openGcode.is_open())
	{
		cout << "Error opening file!";
		return false;
	}

	//首先把Gcode文件全部读进来，保存在inputGcode数组中
	while (getline(openGcode, inputbuffer))
	{
		inputGcode.push_back(inputbuffer);
	}

	//开始对读取的Gcode进行解析
	bool isStartGcode = true;
	bool isMainBody = false;
	bool isEndGcode = false;

	eachLine tmpLine;
	LayerInfo tmpLayer;
	contour tmpContour;
	int contourIndex = 0;

	//逐行对读取的gcode文件进行解析
	for (int i = 0; i < inputGcode.size(); i++)
	{
		inputbuffer = inputGcode.at(i);

		auto isFound = inputbuffer.find("M117 Printing...");
		if (isFound != string::npos)
		{
			//当找到"M117 Printing...",说明这是startGcode的最后一行
			isStartGcode = false;
			isMainBody = true;
		}

		isFound = inputbuffer.find("End GCode");
		if (isFound != string::npos)
		{
			//当找到End GCode表明开始EndGcode部分
			isMainBody = false;
			isEndGcode = true;
		}

		if (isStartGcode)
		{
			//startGcode中的代码直接保存，无需处理
			startGcode.push_back(inputbuffer);
			continue;
		}
		else if (isMainBody)
		{
			//对于mainBody中的代码，进行解析

			//判断是否开始新的一层
			isFound = inputbuffer.find(";LAYER:");
			if (isFound != string::npos)
			{
				//开始新的一层前，先将上一层的contour信息保存
				if (tmpContour.wallOuterContainer.size() != 0)
				{
					tmpLayer.contourContainer.push_back(tmpContour);
					contourIndex = 0;
					//然后将当前contour清空，准备存新的contour
					clearContourContainer(tmpContour);

				}
				//当结束一层时把当前层的信息放入mainbody
				if (tmpLayer.contourContainer.size() != 0)
				{
					mainBody.push_back(tmpLayer);
					//开始新的一层，清空tmpLayer
					tmpLayer.contourContainer.clear();
					tmpLayer.lineContainer.clear();
					tmpLayer.skirtContainer.clear();
					tmpLine.codetype = UNDEFINED;
				}

				//解析当前层的标号
				int cnt_int = 0;
				isFound = inputbuffer.find(':');
				for (int i = isFound + 1; i < inputbuffer.size(); i++)
				{
					cnt_int *= 10;
					cnt_int += inputbuffer[i] - '0';
				}
				tmpLayer.layer = cnt_int;

				tmpLine.codetype = UNDEFINED;
			}

			//判断一层内是否开始一个新的contour
			if (inputbuffer.find(";TYPE:WALL-INNER") != string::npos)
			{
				//在一层内每出现一次wall-inner,表示开始一个新的contour
				//首先将当前contour保存到layerContainer中
				if (tmpContour.wallOuterContainer.size() != 0)
				{
					tmpLayer.contourContainer.push_back(tmpContour);
					//然后将当前contour清空，准备存新的contour
					clearContourContainer(tmpContour);
					//contour的数量增加1
					contourIndex++;

				}
				//更新新的contour
				tmpLine.codetype = WALL_INNER;
				tmpContour.contourIndex = contourIndex;
				tmpContour.typeSequence.push_back(WALL_INNER);

			}
			//判断当前行的代码类型
			else if (inputbuffer.find(";TYPE:WALL-OUTER") != string::npos)
			{
				//对于满足以下特殊条件的也开始新的轮廓
				//对于很短的轮廓，可能不存在wallinner，直接开始wallouter
				if (tmpContour.typeSequence.size() == 0)
				{
					//更新新的contour
					tmpLine.codetype = WALL_OUTER;
					tmpContour.contourIndex = contourIndex;
					tmpContour.typeSequence.push_back(WALL_OUTER);
				}
				else if (tmpContour.typeSequence.at(tmpContour.typeSequence.size() - 1) != WALL_INNER)
				{
					if (tmpContour.wallOuterContainer.size() != 0)
					{
						tmpLayer.contourContainer.push_back(tmpContour);
						//然后将当前contour清空，准备存新的contour
						clearContourContainer(tmpContour);
						//contour的数量增加1
						contourIndex++;
					}
					//更新新的contour
					tmpLine.codetype = WALL_OUTER;
					tmpContour.contourIndex = contourIndex;
					tmpContour.typeSequence.push_back(WALL_OUTER);
				}
				else
				{
					tmpLine.codetype = WALL_OUTER;
					tmpContour.typeSequence.push_back(WALL_OUTER);
				}

			}
			else if (inputbuffer.find(";TYPE:SKIN") != string::npos)
			{
				tmpLine.codetype = SKIN;
				tmpContour.typeSequence.push_back(SKIN);
			}
			else if (inputbuffer.find(";TYPE:FILL") != string::npos)
			{
				tmpLine.codetype = FILL;
				tmpContour.typeSequence.push_back(FILL);
			}
			else if (inputbuffer.find(";TYPE:SUPPORT") != string::npos)
			{
				tmpLine.codetype = SUPPORT;
				tmpContour.typeSequence.push_back(SUPPORT);
			}
			else if (inputbuffer.find(";TYPE:SKIRT") != string::npos)
			{
				tmpLine.codetype = SKIRT;
			}

			//解析行
			parseCommandLine(inputbuffer, tmpLine);

			//存入相应的数组
			if (tmpLine.codetype == WALL_INNER)
				tmpContour.wallInnerContainer.push_back(tmpLine);
			else if (tmpLine.codetype == WALL_OUTER)
				tmpContour.wallOuterContainer.push_back(tmpLine);
			else if (tmpLine.codetype == SKIN)
				tmpContour.skinContainer.push_back(tmpLine);
			else if (tmpLine.codetype == FILL)
				tmpContour.fillContainer.push_back(tmpLine);
			else if (tmpLine.codetype == SUPPORT)
				tmpContour.supportContainer.push_back(tmpLine);
			else if (tmpLine.codetype == SKIRT)
				tmpLayer.skirtContainer.push_back(tmpLine);
			else if (tmpLine.codetype == UNDEFINED)
				tmpLayer.lineContainer.push_back(tmpLine);

			//处理标签为wall-outer时包含多个轮廓的特殊情况
			if (tmpLine.codetype == WALL_OUTER)
			{
				if (isNewContour(inputGcode, i, inputbuffer))
				{
					//从下一行开始新的轮廓
					if (tmpContour.wallOuterContainer.size() != 0)
					{
						tmpLayer.contourContainer.push_back(tmpContour);
						//然后将当前contour清空，准备存新的contour
						clearContourContainer(tmpContour);
						//contour的数量增加1
						contourIndex++;

					}
					//更新新的contour
					tmpLine.codetype = WALL_OUTER;
					tmpContour.contourIndex = contourIndex;
					tmpContour.typeSequence.push_back(WALL_OUTER);

				}

			}

			//清理tmpline
			tmpLine.otherCommand.clear();

		}
		else if (isEndGcode)
		{
			//endGcode中的代码直接保存，无需处理
			endGcode.push_back(inputbuffer);
			continue;
		}
		else
		{
			//如果出现某个代码不属于三类中的任一类，则报错
			cout << "Wrong Gcode type!" << endl;
			return false;
		}
	}
}

bool Gcode::isNewContour(const vector<string> &inputGcode, int index, string inputbuffer)
{
	regex reg1("^G0+.*X+.*Y+.*");//匹配G0
	regex reg2("^G1 F\\d+ E\\d+\.?\\d*");//匹配G1 F E 指令
	regex reg3("^G1+ Z+.*");//匹配G1 Z指令
	regex reg4("^G1 F?\\d* ?X\\d+\.?\\d* Y\\d+\.?\\d* E\\d+\.?\\d*");//匹配正常的G1指令
	smatch r1,r2,r3,r4,r5,r6;
	//满足这一条件的，是在一个wall-outer内有多个轮廓
	//G0 -> G1 E -> G1 Z -> G0 -> G1 Z -> G1 E 
	if (regex_match(inputGcode.at(index), r1, reg1)
		&& regex_match(inputGcode.at(index + 1), r2, reg2)
		&& regex_match(inputGcode.at(index + 2), r3, reg3)
		&& regex_match(inputGcode.at(index + 3), r4, reg1)
		&& regex_match(inputGcode.at(index + 4), r5, reg3)
		&& regex_match(inputGcode.at(index + 5), r6, reg2))
	{
		return true;
	}
	//满足这一条件的，是在一个wall-outer内有多个轮廓
	//G1 X Y E -> G0 -> G1 X Y E
	else if (regex_match(inputGcode.at(index), r1, reg4)
		&& regex_match(inputGcode.at(index + 1), r2, reg1)
		&& regex_match(inputGcode.at(index + 2), r3, reg4))
	{
		return true;
	}
	//暂时只找到这两种pattern下一个wall-outer内有两个轮廓的
	//如果有其他的再加。。
	else
	{
		return false;
	}
	
}

void Gcode::parseCommandLine(string inputbuffer, eachLine &tmpLine)
{
	string tmpString;//用于字符串临时变量的存储
	if (inputbuffer[0] == 'G')
	{
		GCommand tmpGcodeLine;
		tmpLine.isGcommand = true;
		istringstream lineRecord(inputbuffer);
		while (lineRecord >> tmpString)
		{
			switch (tmpString[0])
			{
			case 'G':
				tmpGcodeLine.Gcommand = tmpString;
				continue;
			case 'F':
				tmpString.erase(tmpString.begin());
				tmpGcodeLine.feedRate = std::stof(tmpString);
				continue;
			case 'X':
				tmpString.erase(tmpString.begin());
				tmpGcodeLine.X.isValidValue = true;
				tmpGcodeLine.X.value = std::stof(tmpString);
				continue;
			case 'Y':
				tmpString.erase(tmpString.begin());
				tmpGcodeLine.Y.isValidValue = true;
				tmpGcodeLine.Y.value = std::stof(tmpString);
				continue;
			case 'Z':
				tmpString.erase(tmpString.begin());
				tmpGcodeLine.Z.isValidValue = true;
				tmpGcodeLine.Z.value = std::stof(tmpString);
				zValue.push_back(tmpGcodeLine.Z.value);
				continue;
			case 'E':
				tmpString.erase(tmpString.begin());
				tmpGcodeLine.E.isValidValue = true;
				tmpGcodeLine.E.value = std::stof(tmpString);
				continue;
			default:
				cout << "Gcode line reading error!" << endl;
				break;
			}
		}
		tmpLine.GcommandLine = tmpGcodeLine;

	}
	else
	{
		tmpLine.isGcommand = false;
		tmpLine.otherCommand = inputbuffer;
	}
}

void Gcode::clearContourContainer(contour & tmpContour)
{
	//将tmpContour中的各种数组清空
	tmpContour.contourIndex = -1;
	tmpContour.skinContainer.clear();
	tmpContour.fillContainer.clear();
	tmpContour.supportContainer.clear();
	tmpContour.typeSequence.clear();
	tmpContour.wallInnerContainer.clear();
	tmpContour.wallOuterContainer.clear();

}

bool Gcode::outputGcode(string fileName)
{
	ofstream out(fileName);
	if (!out.good()) {
		std::cout << "Cannot open output file\n";
		return false;
	}
	//输出startgcode
	for (int i = 0; i < startGcode.size(); i++)
	{
		out << startGcode.at(i) << endl;
	}

	//输出gcode主体
	for (int i = 0; i < mainBody.size(); i++)
	{
		out << "Layer:" << i << endl;
		out << "G0 Z" << zValue.at(i) << endl;
		LayerInfo tmpLayer = mainBody.at(i);
		//输出当前层的其他指令
		for (int j = 0; j < tmpLayer.lineContainer.size(); j++)
		{
			eachLine tmpline = tmpLayer.lineContainer.at(j);
			outputLine(tmpline, out);
		}
		if (tmpLayer.skirtContainer.size()>0)
		{
			for (int j = 0; j < tmpLayer.skirtContainer.size(); j++)
			{
				eachLine tmpline = tmpLayer.skirtContainer.at(j);
				outputLine(tmpline, out);
			}
		}
		//输出当前层内的各个轮廓
		for (int j = 0; j <tmpLayer.contourContainer.size(); j++)
		{
			contour tmpcontour = tmpLayer.contourContainer.at(j);
			for (int p = 0; p < tmpcontour.typeSequence.size(); p++)
			{
				switch (tmpcontour.typeSequence.at(p))
				{
				case WALL_INNER:
				{
					for (int k = 0; k < tmpcontour.wallInnerContainer.size(); k++)
					{
						eachLine tmpline = tmpcontour.wallInnerContainer.at(k);
						outputLine(tmpline, out);
					}
					break;
				}
				case WALL_OUTER:
				{
					for (int k = 0; k < tmpcontour.wallOuterContainer.size(); k++)
					{
						eachLine tmpline = tmpcontour.wallOuterContainer.at(k);
						outputLine(tmpline, out);
					}
					break;
				}
				case FILL:
				{
					for (int k = 0; k < tmpcontour.fillContainer.size(); k++)
					{
						eachLine tmpline = tmpcontour.fillContainer.at(k);
						outputLine(tmpline, out);
					}
					break;
				}
				case SKIN:
				{
					for (int k = 0; k < tmpcontour.skinContainer.size(); k++)
					{
						eachLine tmpline = tmpcontour.skinContainer.at(k);
						outputLine(tmpline, out);
					}
					break;
				}
				case SUPPORT:
				{
					for (int k = 0; k < tmpcontour.supportContainer.size(); k++)
					{
						eachLine tmpline = tmpcontour.supportContainer.at(k);
						outputLine(tmpline, out);
					}
					break;
				}
				default:
					break;
				}
			}

		}
	}

	//输出endgcode
	for (int i = 0; i < endGcode.size(); i++)
	{
		out << endGcode.at(i) << endl;
	}

	return true;
}

bool Gcode::outputGcodeShell(string fileName)
{


	return false;
}

void Gcode::outputLine(eachLine tmpline, ofstream &out)
{
	if (!tmpline.isGcommand)
	{
		out << tmpline.otherCommand << endl;
	}
	else if (tmpline.isGcommand)
	{
		out << tmpline.GcommandLine.Gcommand << " ";

		if (tmpline.GcommandLine.feedRate != -1)
		{
			out << "F" << tmpline.GcommandLine.feedRate << " ";
		}
		else
		{
			if (tmpline.GcommandLine.Gcommand == "G0")
				out << "F" << "4800 ";
			else if (tmpline.GcommandLine.Gcommand == "G1")
				out << "F" << "1200 ";
		}

		if (tmpline.GcommandLine.X.isValidValue)
		{
			out << "X" << tmpline.GcommandLine.X.value << " ";
		}
		if (tmpline.GcommandLine.Y.isValidValue)
		{
			out << "Y" << tmpline.GcommandLine.Y.value << " ";
		}
		if (tmpline.GcommandLine.Z.isValidValue)
		{
			out << "Z" << tmpline.GcommandLine.Z.value << " ";
		}
		if (tmpline.GcommandLine.E.isValidValue)
		{
			//对E保留小数点后5位
			out.precision(5);
			out.setf(ios::fixed);
			out << "E" << tmpline.GcommandLine.E.value;
			//对于其他的保留小数点后3位
			out.precision(3);
		}
		out << endl;
	}
}

void Gcode::updateGcode()
{
	vector<LayerInfo> newMainBody;

	for (int i = 0; i < mainBody.size(); i++)
	{
		LayerInfo newTmpLayer;
		newTmpLayer.layer = mainBody.at(i).layer;
		newTmpLayer.lineContainer = mainBody.at(i).lineContainer;
		newTmpLayer.skirtContainer = mainBody.at(i).skirtContainer;

		for (int j = 0; j < mainBody.at(i).contourContainer.size(); j++)
		{
			contour newTmpContour;
			//将一个contour中除wallouter以外的其他部分保留
			newTmpContour.contourIndex = mainBody.at(i).contourContainer.at(j).contourIndex;
			newTmpContour.typeSequence = mainBody.at(i).contourContainer.at(j).typeSequence;
			newTmpContour.fillContainer = mainBody.at(i).contourContainer.at(j).fillContainer;
			newTmpContour.skinContainer = mainBody.at(i).contourContainer.at(j).skinContainer;
			newTmpContour.supportContainer = mainBody.at(i).contourContainer.at(j).supportContainer;
			newTmpContour.wallInnerContainer = mainBody.at(i).contourContainer.at(j).wallInnerContainer;

			//只处理wallouter部分
			for (int k = 0; k < mainBody.at(i).contourContainer.at(j).wallOuterContainer.size(); k++)
			{
				eachLine tmpline = mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k);
				bool isDelete = false;
				//只对G1指令中外壳部分进行操作
				if (tmpline.isGcommand
					&& tmpline.GcommandLine.Gcommand == "G1"
					&& tmpline.GcommandLine.X.isValidValue
					&& tmpline.GcommandLine.Y.isValidValue
					&& tmpline.GcommandLine.E.isValidValue)
				{
					//对于有sample point的层
					if (i < samplePoints.size())
					{
						//检查在sample point中有没有在这一行的
						for (int index = 0; index < samplePoints[i][j].size(); index++)
						{
							//在当前轮廓的sample point
							if (samplePoints[i][j][index].contourIndex == j
								&& samplePoints[i][j][index].indexStart == k)
							{
								//将当前行的属性拷贝至新添加的行
								eachLine addLine;
								addLine.codetype = tmpline.codetype;
								addLine.isGcommand = tmpline.isGcommand;
								addLine.GcommandLine.Gcommand = tmpline.GcommandLine.Gcommand;
								addLine.GcommandLine.feedRate = tmpline.GcommandLine.feedRate;
								addLine.GcommandLine.Z = tmpline.GcommandLine.Z;

								//将sample point点的坐标值复制到新添加行
								addLine.GcommandLine.X.value = samplePoints[i][j][index].x;
								addLine.GcommandLine.Y.value = samplePoints[i][j][index].y;
								addLine.GcommandLine.E.value = samplePoints[i][j][index].e;
								addLine.GcommandLine.E.isValidValue = true;
								addLine.GcommandLine.X.isValidValue = true;
								addLine.GcommandLine.Y.isValidValue = true;

								newTmpContour.wallOuterContainer.push_back(addLine);
								isDelete = true;
							}

						}
					}
					else//对于没有sample point的层，不进行处理直接保存进新的数组中
					{
						newTmpContour.wallOuterContainer.push_back(mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k));
					}

				}
				else//对于不满足外轮廓上G1的其他指令，不进行处理直接保存进新的数组中
				{
					newTmpContour.wallOuterContainer.push_back(mainBody.at(i).contourContainer.at(j).wallOuterContainer.at(k));
				}
			}
			//一层内的一个轮廓处理完毕
			newTmpLayer.contourContainer.push_back(newTmpContour);
			//清理vector
			newTmpContour.wallOuterContainer.clear();
		}
		//一层处理完毕
		newMainBody.push_back(newTmpLayer);

	}
	//最后用新生成的信息替代原代码
	mainBody = newMainBody;
}

point2D Gcode::findActuralStartPoint(Gcode currGcode, int layerBeforeIdx, int currLayerIdx, int contourIndex)
{
	point2D acturalStartPoint;
	LayerInfo tmplayer = currGcode.mainBody.at(currLayerIdx);
	contour tmpContour = tmplayer.contourContainer.at(contourIndex);
	eachLine tmpLine;
	int validBaseG1;
	int validEG1;
	int validXYG0;
	bool isG1Find = false;
	bool isG0Found = false;

	//1. 找到离wallouter标识符下第一个X,Y,E值均有效的G1指令
	for (int i = 0; i < tmpContour.wallOuterContainer.size(); i++)
	{
		tmpLine = tmpContour.wallOuterContainer.at(i);
		if (tmpLine.isGcommand
			&& tmpLine.GcommandLine.X.isValidValue == true
			&& tmpLine.GcommandLine.Y.isValidValue == true
			&& tmpLine.GcommandLine.E.isValidValue == true)
		{
			//找到后记录该点的位置
			validBaseG1 = i;
			break;
		}
	}

	//2. 从该指令向前，找到离他最近的E有效的G1指令,这一步是为了确定E
	//2.1 第一种情况是在wallouter中可以找到
	for (int i = validBaseG1 - 1; i != -1; i--)//从该点的前一个点开始找
	{
		tmpLine = tmpContour.wallOuterContainer.at(i);
		if (tmpLine.isGcommand
			&& tmpLine.GcommandLine.E.isValidValue == true)
		{
			isG1Find = true;
			acturalStartPoint.e = tmpLine.GcommandLine.E.value;
			break;
		}
	}
	//2.2 第二种情况是在wallouter中找不到
	//需要在当前轮廓内找
	if (isG1Find == false
		&& tmpContour.typeSequence.at(0) != WALL_OUTER)
	{
		//首先找到WALL_OUTER的位置
		for (int i = 0; i < tmpContour.typeSequence.size(); i++)
		{
			if (tmpContour.typeSequence.at(i) == WALL_OUTER)
				validEG1 = i - 1;
		}
		if (tmpContour.typeSequence.at(validEG1) == WALL_INNER)
		{
			for (int j = tmpContour.wallInnerContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.wallInnerContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					isG1Find = true;
					acturalStartPoint.e = tmpLine.GcommandLine.E.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validEG1) == FILL)
		{
			for (int j = tmpContour.fillContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.fillContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					isG1Find = true;
					acturalStartPoint.e = tmpLine.GcommandLine.E.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validEG1) == SKIN)
		{
			for (int j = tmpContour.skinContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.skinContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					isG1Find = true;
					acturalStartPoint.e = tmpLine.GcommandLine.E.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validEG1) == SUPPORT)
		{
			for (int j = tmpContour.supportContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.supportContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					isG1Find = true;
					acturalStartPoint.e = tmpLine.GcommandLine.E.value;
					break;
				}
			}
		}
	}
	//2.3 第三种情况是在当前轮廓内找不到
	//则需要在当前层向前找
	if (isG1Find == false
		&& tmpContour.typeSequence.at(0) == WALL_OUTER)
	{
		//在当前层的lineContainer中查找
		int tmpIndx = tmplayer.lineContainer.size() - 1;
		for (int i = tmpIndx; i > 0; i--)
		{
			tmpLine = tmplayer.lineContainer.at(i);
			if (tmpLine.isGcommand
				&& tmpLine.GcommandLine.E.isValidValue == true)
			{
				isG1Find = true;
				acturalStartPoint.e = tmpLine.GcommandLine.E.value;
				break;
			}
		}
	}
	//2.4 在当前层内找不到要向前一层找
	//这种情况出现在使用only mesh surface模式下
	//此时，向前一层的最后一个外轮廓找
	if (isG1Find == false
		&& currLayerIdx == 0)
	{
		//如果当前层是第一层，那么从E=0开始
		isG1Find = true;
		acturalStartPoint.e = 0;
	}
	else if (isG1Find == false
		&& currLayerIdx != 0)
	{
		LayerInfo lastLayer = currGcode.mainBody.at(layerBeforeIdx);
		contour lastContour = lastLayer.contourContainer.at(lastLayer.contourContainer.size() - 1);
		for (int i = lastContour.wallOuterContainer.size() - 1; i > -1; i--)
		{
			tmpLine = lastContour.wallOuterContainer.at(i);
			if (tmpLine.isGcommand
				&& tmpLine.GcommandLine.Gcommand == "G1"
				&& tmpLine.GcommandLine.E.isValidValue == true)
			{
				isG1Find = true;
				acturalStartPoint.e = tmpLine.GcommandLine.E.value;
				break;
			}
		}
	}
	
	//3. 从该指令向前，找到离他最近的X，Y有效的G0指令，这一步是为了确定X，Y
	//3.1 第一种情况是在wallouter中可以找到
	for (int i = validBaseG1 - 1; i != -1; i--)//从该点的前一个点开始找
	{
		tmpLine = tmpContour.wallOuterContainer.at(i);
		if (tmpLine.isGcommand
			&& tmpLine.GcommandLine.Gcommand == "G0"
			&& tmpLine.GcommandLine.X.isValidValue == true
			&& tmpLine.GcommandLine.Y.isValidValue == true)
		{
			isG0Found = true;
			acturalStartPoint.x = tmpLine.GcommandLine.X.value;
			acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
			break;
		}
	}
	//3.2 第二种情况是在wallouter中找不到
	//需要在当前轮廓内找
	if (isG0Found == false
		&& tmpContour.typeSequence.at(0) != WALL_OUTER)
	{
		//首先找到WALL_OUTER的位置
		for (int i = 0; i < tmpContour.typeSequence.size(); i++)
		{
			if (tmpContour.typeSequence.at(i) == WALL_OUTER)
				validXYG0 = i - 1;
		}
		if (tmpContour.typeSequence.at(validXYG0) == WALL_INNER)
		{
			for (int j = tmpContour.wallInnerContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.wallInnerContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.Gcommand == "G0"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true)
				{
					isG0Found = true;
					acturalStartPoint.x = tmpLine.GcommandLine.X.value;
					acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validXYG0) == FILL)
		{
			for (int j = tmpContour.fillContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.fillContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.Gcommand == "G0"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true)
				{
					isG0Found = true;
					acturalStartPoint.x = tmpLine.GcommandLine.X.value;
					acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validXYG0) == SKIN)
		{
			for (int j = tmpContour.skinContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.skinContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.Gcommand == "G0"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true)
				{
					isG0Found = true;
					acturalStartPoint.x = tmpLine.GcommandLine.X.value;
					acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validXYG0) == SUPPORT)
		{
			for (int j = tmpContour.supportContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.supportContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.Gcommand == "G0"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true)
				{
					isG0Found = true;
					acturalStartPoint.x = tmpLine.GcommandLine.X.value;
					acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
					break;
				}
			}
		}
	}

	//3.3 第三种情况是在当前轮廓内找不到
	//则需要在当前层向前找
	//如果当前轮廓是一层中的第一个轮廓,则在当前层的lineContainer中查找
	//else if (isG1Find == false
	else if (isG0Found == false
		&& tmpContour.typeSequence.at(0) == WALL_OUTER
		&& contourIndex == 0)
	{
		//在当前层的lineContainer中查找
		int tmpIndx = tmplayer.lineContainer.size() - 1;
		for (int i = tmpIndx; i > 0; i--)
		{
			tmpLine = tmplayer.lineContainer.at(i);
			if (tmpLine.isGcommand
				&& tmpLine.GcommandLine.Gcommand == "G0"
				&& tmpLine.GcommandLine.X.isValidValue == true
				&& tmpLine.GcommandLine.Y.isValidValue == true)
			{
				isG1Find = true;
				acturalStartPoint.x = tmpLine.GcommandLine.X.value;
				acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
				break;
			}
		}
	}
	//如果当前轮廓是不是一层中的第一个轮廓，则向之前的轮廓中找
	//暂时做一个简化，直接找前一个轮廓的wall-outer最后一个点
	//else if (isG1Find == false
	//	&& tmpContour.typeSequence.at(0) == WALL_OUTER
	//	&& contourIndex != 0)
	//{
	//	tmpContour = tmpLayer.contourContainer.at(contourIndex - 1);
	//	tmpLine = tmpContour.wallOuterContainer.at(tmpContour.wallOuterContainer.size() - 1);
	//	
	//	isG1Find = true;
	//	acturalStartPoint.x = tmpLine.GcommandLine.X.value;
	//	acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
	//}
	
	return acturalStartPoint;
}


point2D Gcode::findActuralStartXY(LayerInfo currLayer, int contourIndex)
{
	point2D acturalStartPoint;
	contour tmpContour = currLayer.contourContainer.at(contourIndex);
	eachLine tmpLine;
	int validBaseG1;
	int validEG1;
	int validXYG0;
	bool isG1Find = false;
	bool isG0Found = false;

	//1. 找到离wallouter标识符下第一个X,Y,E值均有效的G1指令
	for (int i = 0; i < tmpContour.wallOuterContainer.size(); i++)
	{
		tmpLine = tmpContour.wallOuterContainer.at(i);
		if (tmpLine.isGcommand
			&& tmpLine.GcommandLine.X.isValidValue == true
			&& tmpLine.GcommandLine.Y.isValidValue == true
			&& tmpLine.GcommandLine.E.isValidValue == true)
		{
			//找到后记录该点的位置
			validBaseG1 = i;
			break;
		}
	}

	//2. 从该指令向前，找到离他最近的E有效的G1指令,这一步是为了确定E
	//2.1 第一种情况是在wallouter中可以找到
	for (int i = validBaseG1 - 1; i != -1; i--)//从该点的前一个点开始找
	{
		tmpLine = tmpContour.wallOuterContainer.at(i);
		if (tmpLine.isGcommand
			&& tmpLine.GcommandLine.E.isValidValue == true)
		{
			isG1Find = true;
			acturalStartPoint.e = tmpLine.GcommandLine.E.value;
			break;
		}
	}
	//2.2 第二种情况是在wallouter中找不到
	//需要在当前轮廓内找
	if (isG1Find == false
		&& tmpContour.typeSequence.at(0) != WALL_OUTER)
	{
		//首先找到WALL_OUTER的位置
		for (int i = 0; i < tmpContour.typeSequence.size(); i++)
		{
			if (tmpContour.typeSequence.at(i) == WALL_OUTER)
				validEG1 = i - 1;
		}
		if (tmpContour.typeSequence.at(validEG1) == WALL_INNER)
		{
			for (int j = tmpContour.wallInnerContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.wallInnerContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					isG1Find = true;
					acturalStartPoint.e = tmpLine.GcommandLine.E.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validEG1) == FILL)
		{
			for (int j = tmpContour.fillContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.fillContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					isG1Find = true;
					acturalStartPoint.e = tmpLine.GcommandLine.E.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validEG1) == SKIN)
		{
			for (int j = tmpContour.skinContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.skinContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					isG1Find = true;
					acturalStartPoint.e = tmpLine.GcommandLine.E.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validEG1) == SUPPORT)
		{
			for (int j = tmpContour.supportContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.supportContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					isG1Find = true;
					acturalStartPoint.e = tmpLine.GcommandLine.E.value;
					break;
				}
			}
		}
	}
	//2.3 第三种情况是在当前轮廓内找不到
	//则需要在当前层向前找
	else if (isG1Find == false
		&& tmpContour.typeSequence.at(0) == WALL_OUTER)
	{
		//在当前层的lineContainer中查找
		int tmpIndx = currLayer.lineContainer.size() - 1;
		for (int i = tmpIndx; i > 0; i--)
		{
			tmpLine = currLayer.lineContainer.at(i);
			if (tmpLine.isGcommand
				&& tmpLine.GcommandLine.E.isValidValue == true)
			{
				isG1Find = true;
				acturalStartPoint.e = tmpLine.GcommandLine.E.value;
				break;
			}
		}
	}

	//3. 从该指令向前，找到离他最近的X，Y有效的G0指令，这一步是为了确定X，Y
	//3.1 第一种情况是在wallouter中可以找到
	for (int i = validBaseG1 - 1; i != -1; i--)//从该点的前一个点开始找
	{
		tmpLine = tmpContour.wallOuterContainer.at(i);
		if (tmpLine.isGcommand
			&& tmpLine.GcommandLine.Gcommand == "G0"
			&& tmpLine.GcommandLine.X.isValidValue == true
			&& tmpLine.GcommandLine.Y.isValidValue == true)
		{
			isG0Found = true;
			acturalStartPoint.x = tmpLine.GcommandLine.X.value;
			acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
			break;
		}
	}
	//3.2 第二种情况是在wallouter中找不到
	//需要在当前轮廓内找
	if (isG0Found == false
		&& tmpContour.typeSequence.at(0) != WALL_OUTER)
	{
		//首先找到WALL_OUTER的位置
		for (int i = 0; i < tmpContour.typeSequence.size(); i++)
		{
			if (tmpContour.typeSequence.at(i) == WALL_OUTER)
				validXYG0 = i - 1;
		}
		if (tmpContour.typeSequence.at(validXYG0) == WALL_INNER)
		{
			for (int j = tmpContour.wallInnerContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.wallInnerContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.Gcommand == "G0"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true)
				{
					isG0Found = true;
					acturalStartPoint.x = tmpLine.GcommandLine.X.value;
					acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validXYG0) == FILL)
		{
			for (int j = tmpContour.fillContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.fillContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.Gcommand == "G0"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true)
				{
					isG0Found = true;
					acturalStartPoint.x = tmpLine.GcommandLine.X.value;
					acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validXYG0) == SKIN)
		{
			for (int j = tmpContour.skinContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.skinContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.Gcommand == "G0"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true)
				{
					isG0Found = true;
					acturalStartPoint.x = tmpLine.GcommandLine.X.value;
					acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
					break;
				}
			}
		}
		else if (tmpContour.typeSequence.at(validXYG0) == SUPPORT)
		{
			for (int j = tmpContour.supportContainer.size() - 1; j >= 0; j--)
			{
				tmpLine = tmpContour.supportContainer.at(j);
				if (tmpLine.isGcommand
					&& tmpLine.GcommandLine.Gcommand == "G0"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true)
				{
					isG0Found = true;
					acturalStartPoint.x = tmpLine.GcommandLine.X.value;
					acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
					break;
				}
			}
		}
	}

	//3.3 第三种情况是在当前轮廓内找不到
	//则需要在当前层向前找
	//如果当前轮廓是一层中的第一个轮廓,则在当前层的lineContainer中查找
	//else if (isG1Find == false
	else if (isG0Found == false
		&& tmpContour.typeSequence.at(0) == WALL_OUTER
		&& contourIndex == 0)
	{
		//在当前层的lineContainer中查找
		int tmpIndx = currLayer.lineContainer.size() - 1;
		for (int i = tmpIndx; i > 0; i--)
		{
			tmpLine = currLayer.lineContainer.at(i);
			if (tmpLine.isGcommand
				&& tmpLine.GcommandLine.Gcommand == "G0"
				&& tmpLine.GcommandLine.X.isValidValue == true
				&& tmpLine.GcommandLine.Y.isValidValue == true)
			{
				isG1Find = true;
				acturalStartPoint.x = tmpLine.GcommandLine.X.value;
				acturalStartPoint.y = tmpLine.GcommandLine.Y.value;
				break;
			}
		}
	}

	return acturalStartPoint;
}


point2D Gcode::findActuralEndPoint(LayerInfo tmpLayer, int contourIndex)
{
	point2D acturalEndPoint;
	int tmpIndexEnd = tmpLayer.contourContainer.at(contourIndex).wallOuterContainer.size() - 1;
	for (tmpIndexEnd; tmpIndexEnd > 0; tmpIndexEnd--)
	{
		eachLine tmpline = tmpLayer.contourContainer.at(contourIndex).wallOuterContainer.at(tmpIndexEnd);
		if (tmpline.isGcommand
			&& tmpline.GcommandLine.Gcommand == "G1"
			&& tmpline.codetype == WALL_OUTER
			&& tmpline.GcommandLine.X.isValidValue
			&& tmpline.GcommandLine.Y.isValidValue
			&& tmpline.GcommandLine.E.isValidValue)
		{
			acturalEndPoint.e = tmpline.GcommandLine.E.value;
			break;
		}
	}

	return acturalEndPoint;
}

void Gcode::prepareOuterlines(Gcode currGcode, vector<point3D> &outputPoints, vector<int> &outContourIdx)
{
	bool isFirstPoint;
	point2D tmpStartPoint;
	point3D tmpPoint;
	int outpointCount = 1;

	for (int i = 0; i < currGcode.mainBody.size(); i++)
	{
		LayerInfo tmplayer = mainBody.at(i);
		for (int j = 0; j < tmplayer.contourContainer.size(); j++)
		{
			contour tmpContour = tmplayer.contourContainer.at(j);
			isFirstPoint = true;
			for (int k = 0; k < tmpContour.wallOuterContainer.size(); k++)
			{
				if (isFirstPoint == true)
				{
					tmpStartPoint = findActuralStartPoint(currGcode, i - 1, i, j);

					tmpPoint.x = tmpStartPoint.x;
					tmpPoint.y = tmpStartPoint.y;
					tmpPoint.z = zValue.at(i);

					outputPoints.push_back(tmpPoint);
					outpointCount++;
					isFirstPoint = false;
				}
				eachLine tmpLine = tmpContour.wallOuterContainer.at(k);
				if (tmpLine.isGcommand == true
					&& tmpLine.GcommandLine.Gcommand == "G1"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					tmpPoint.x = tmpStartPoint.x;
					tmpPoint.y = tmpStartPoint.y;
					tmpPoint.z = zValue.at(i);

					outputPoints.push_back(tmpPoint);
					outpointCount++;
				}
			}
			//每处理完一个轮廓，记录该轮廓结尾顶点的标号
			outContourIdx.push_back(outpointCount);
		}

	}
}

void Gcode::prepareOuterLines(Gcode currGcode, vector<vector<point3D>>& outputPoints)
{
	bool isFirstPoint;
	point2D tmpStartPoint;
	point3D tmpPoint;
	vector<point3D> contourPoints;

	for (int i = 0; i < currGcode.mainBody.size(); i++)
	{
		LayerInfo tmplayer = mainBody.at(i);
		for (int j = 0; j < tmplayer.contourContainer.size(); j++)
		{
			contour tmpContour = tmplayer.contourContainer.at(j);
			isFirstPoint = true;
			for (int k = 0; k < tmpContour.wallOuterContainer.size(); k++)
			{
				if (isFirstPoint == true)
				{
					tmpStartPoint = findActuralStartPoint(currGcode, i - 1, i, j);

					tmpPoint.x = tmpStartPoint.x;
					tmpPoint.y = tmpStartPoint.y;
					tmpPoint.z = zValue.at(i);

					contourPoints.push_back(tmpPoint);
					isFirstPoint = false;
				}
				eachLine tmpLine = tmpContour.wallOuterContainer.at(k);
				if (tmpLine.isGcommand == true
					&& tmpLine.GcommandLine.Gcommand == "G1"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					tmpPoint.x = tmpLine.GcommandLine.X.value;
					tmpPoint.y = tmpLine.GcommandLine.Y.value;
					tmpPoint.z = zValue.at(i);

					contourPoints.push_back(tmpPoint);
				}
			}
			//每处理完一个轮廓，将该轮廓包含的点存入
			outputPoints.push_back(contourPoints);
			contourPoints.clear();
		}
	}
}

void Gcode::prepareModelPolygon(Gcode currGcode, vector<vector<vector<point2D>>>& modelPolygon)
{
	point2D tmpPoint;
	vector<point2D> contourPoints;
	vector<vector<point2D>> layerContours;

	for (int i = 0; i < currGcode.mainBody.size(); i++)
	{
		LayerInfo tmplayer = mainBody.at(i);
		for (int j = 0; j < tmplayer.contourContainer.size(); j++)
		{
			contour tmpContour = tmplayer.contourContainer.at(j);
			for (int k = 0; k < tmpContour.wallInnerContainer.size(); k++)
			{
				eachLine tmpLine = tmpContour.wallInnerContainer.at(k);
				if (tmpLine.isGcommand == true
					&& tmpLine.GcommandLine.Gcommand == "G1"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					tmpPoint.x = tmpLine.GcommandLine.X.value;
					tmpPoint.y = tmpLine.GcommandLine.Y.value;

					contourPoints.push_back(tmpPoint);
				}
			}
			//每处理完一个轮廓，将该轮廓包含的点存入
			layerContours.push_back(contourPoints);
			contourPoints.clear();
		}
		//处理完一层，将当前层的点存入
		modelPolygon.push_back(layerContours);
		layerContours.clear();
	}
}

void Gcode::smoothGcodeWallouter(vector<vector<point3D>> &outputPoints)
{
	point3D prePoint, currPoint, nextPoint;

	int beginContourIdx, endContourIdx;
	int iterCount = 2;//平滑处理的次数

	vector<vector<point3D>> copyOutputPoints = outputPoints;
	vector<point3D> smoothContour;

	vector<point3D> pointsOneContour;
	while (iterCount>0)
	{
		for (int i = 0; i < copyOutputPoints.size(); i++)
		{
			pointsOneContour = copyOutputPoints.at(i);
			for (int j = 0; j < pointsOneContour.size(); j++)
			{
				currPoint = pointsOneContour.at(j);
				beginContourIdx = 0;
				endContourIdx = pointsOneContour.size() - 1;

				if (j == beginContourIdx)
				{
					prePoint = pointsOneContour.at(endContourIdx);
					nextPoint = pointsOneContour.at(j + 1);
				}
				else if (j == endContourIdx)
				{
					prePoint = pointsOneContour.at(j - 1);
					nextPoint = pointsOneContour.at(beginContourIdx);
				}
				else
				{
					prePoint = pointsOneContour.at(j - 1);
					nextPoint = pointsOneContour.at(j + 1);
				}

				//平滑处理该点的坐标
				currPoint = smoothPoints(currPoint, prePoint, nextPoint);
				smoothContour.push_back(currPoint);
			}
			copyOutputPoints.at(i) = smoothContour;
			smoothContour.clear();
		}
		iterCount--;
	}
	
	//输出平滑后的点
	outputPoints = copyOutputPoints;
}

point3D Gcode::smoothPoints(point3D currPoint, point3D prePoint, point3D nextPoint)
{
	currPoint.x = 0.5*currPoint.x + 0.25*prePoint.x + 0.25*nextPoint.x;
	currPoint.y = 0.5*currPoint.y + 0.25*prePoint.y + 0.25*nextPoint.y;

	return currPoint;
}

void Gcode::outputLineSegmentsForRendering(Gcode currGcode, string fileName)
{
	ofstream out(fileName);
	if (!out.good()) {
		std::cout << "Cannot open output file\n";
		return;
	}

	bool isFirstPoint = true;
	point2D tmpStartPoint;

	//输出线段
	for (int i = 0; i < currGcode.mainBody.size(); i++)
	{
		LayerInfo tmplayer = mainBody.at(i);
		out << "layer:" << i << endl;
		for (int j = 0; j < tmplayer.contourContainer.size(); j++)
		{
			contour tmpContour = tmplayer.contourContainer.at(j);
			out << "contour:" << j << endl;
			isFirstPoint = true;
			for (int k = 0; k < tmpContour.wallOuterContainer.size(); k++)
			{
				if (isFirstPoint == true)
				{
					tmpStartPoint = findActuralStartPoint(currGcode, i - 1, i, j);
					out << tmpStartPoint.x << " " << tmpStartPoint.y << " " << zValue.at(i) << endl;
					isFirstPoint = false;
				}
				eachLine tmpLine = tmpContour.wallOuterContainer.at(k);
				if (tmpLine.isGcommand == true
					&& tmpLine.GcommandLine.Gcommand == "G1"
					&& tmpLine.GcommandLine.X.isValidValue == true
					&& tmpLine.GcommandLine.Y.isValidValue == true
					&& tmpLine.GcommandLine.E.isValidValue == true)
				{
					out << tmpLine.GcommandLine.X.value << " " << tmpLine.GcommandLine.Y.value << " " << zValue.at(i) << endl;
				}

			}
		}
	}
	cout << "output finish!" << endl;
}

void Gcode::outputLineObj(Gcode currGcode, string fileName)
{
	vector<point3D> outputPoints;
	vector<int> outContourIdx;

	//准备输出的线段
	prepareOuterlines(currGcode, outputPoints, outContourIdx);

	ofstream out(fileName);
	if (!out.good()) {
		std::cout << "Cannot open output file\n";
		return;
	}

	bool isFirstPoint = true;
	point2D tmpStartPoint;
	int outpointCount = 1;

	//输出线段
	for (int i = 0; i < outputPoints.size(); i++)
	{
		out << "v "<< outputPoints.at(i).x << " " << outputPoints.at(i).y << " " << outputPoints.at(i).z << endl;
	}

	isFirstPoint = true;
	for (int idx = 0; idx < outContourIdx.size(); idx++)
	{
		if (isFirstPoint)
		{
			out << "l ";
			for (int idxy = 1; idxy < outContourIdx.at(idx); idxy++)
			{
				out << idxy << " ";
			}
			out<< "1" << endl;//重新输出起点，使收尾相连
			isFirstPoint = false;
		}
		else
		{
			out << "l ";
			for (int idxy = outContourIdx.at(idx-1); idxy < outContourIdx.at(idx); idxy++)
			{
				out << idxy << " ";
			}
			//重新输出起点，使收尾相连
			out<< outContourIdx.at(idx - 1) << endl;
		}

	}

	cout << "output finish!" << endl;
}

void Gcode::outputLineObj2(Gcode currGcode, string fileName)
{
	vector<vector<point3D>> outputPoints;
	vector<int> outContourIdx;

	//准备输出的线段
	prepareOuterLines(currGcode, outputPoints);
	smoothGcodeWallouter(outputPoints);

	ofstream out(fileName);
	if (!out.good()) {
		std::cout << "Cannot open output file\n";
		return;
	}

	bool isFirstPoint = true;
	point2D tmpStartPoint;
	int outpointCount = 1;

	//输出点
	for (int i = 0; i < outputPoints.size(); i++)
	{
		for (int j = 0; j < outputPoints.at(i).size(); j++)
		{
			out << "v " << outputPoints.at(i).at(j).x << " " 
				<< outputPoints.at(i).at(j).y << " " << outputPoints.at(i).at(j).z << endl;
		}
	}
	//输出线
	int num = 1;
	for (int i = 0; i < outputPoints.size() + 1; i++)
	{	
		out << "l ";
		for (int j = num; j <num + outputPoints.at(i).size(); j++)
		{
			out << j << " ";
		}
		out << num << endl;
		//计算已经输出了多少个点
		num = num + outputPoints.at(i).size();

	}
}

void Gcode::generateZvalue(Gcode & tmpGcode)
{
	for (int i = 0; i < tmpGcode.mainBody.size(); i++)
	{
		tmpGcode.zValue.push_back(i*0.3);
	}
	
}
