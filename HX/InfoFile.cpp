#include "pch.h"
#include "InfoFile.h"


CInfoFile::CInfoFile()
{
}


CInfoFile::~CInfoFile()
{
}

//��ȡ��½��Ϣ
void CInfoFile::ReadLogin(CString &name, CString &pwd)
{
	ifstream ifs; //�����ļ��������
	ifs.open(_F_LOGIN); //���ļ�

	char buf[1024] = { 0 };

	ifs.getline(buf, sizeof(buf)); //��ȡһ������
	name = CString(buf);			 //char *ת��ΪCString

	ifs.getline(buf, sizeof(buf));
	pwd = CString(buf);

	ifs.close(); //�ر��ļ�
}

//�޸�����
void CInfoFile::WritePwd(char* name, char* pwd)
{
	ofstream ofs;	 //�����ļ��������
	ofs.open(_F_LOGIN); //���ļ�

	ofs << name << endl; //nameд���ļ�
	ofs << pwd << endl;	//pwdд���ļ�

	ofs.close();	//�ر��ļ�
}

//��ȡ��Ʒ��Ϣ
void CInfoFile::ReadDocline(CString &type,double &x_floor, double &x_ceil, double &y_floor, double &y_ceil, double &theta_floor, double &theta_ceil)
{
	std::vector<CString> vecResult;
	std::vector<CString> strTmp;
	//msg tmp;
	CString str;//���ڽ��շָ��ַ�����ʱ����
	CString strline;
	
	//��ȡ�����趨
	char* old_locale = _strdup(setlocale(LC_CTYPE, NULL));
	//�趨���������ġ�����
	setlocale(LC_CTYPE, "chs");
	CStdioFile file;//����һ��CStdioFile��Ķ��� file
	BOOL flag = file.Open(_T(".\\stock.txt"), CFile::modeRead);//open������Ҫ������������ǰһ�����ļ�·������һ�����ļ��Ĵ�ģʽ
	if (flag == FALSE)
	{
		//MessageBox(_T("�ļ���ʧ��"));
	}
	while (file.ReadString(strline))
	{
		vecResult.push_back(strline);
	}
	file.Close();
	//�ָ������趨
	setlocale(LC_CTYPE, old_locale);
	free(old_locale);
	//�ָ��ַ�����//

	
	int curPos = 0;
	str = vecResult[0].Tokenize(_T(" "), curPos);
	while (str.Trim() != _T(""))
	{
		strTmp.push_back(str);
		str = vecResult[0].Tokenize(_T(" "), curPos);
	}
	type = strTmp[0];
	x_floor = _wtof(strTmp[1]);
	x_ceil = _wtof(strTmp[2]);
	y_floor = _wtof(strTmp[3]);
	y_ceil = _wtof(strTmp[4]);
	theta_floor = _wtof(strTmp[5]);
	theta_ceil = _wtof(strTmp[6]);
	/*tmp.frame_length = _wtof(strTmp[7]);
	tmp.frame_width = _wtof(strTmp[8]);
	tmp.image_threshold = _wtof(strTmp[9]);*/

}

//д���ļ�
void CInfoFile::WirteDocline(CString &type, double &x_floor, double &x_ceil, double &y_floor, double &y_ceil, double &theta_floor, double &theta_ceil)
{
	
	

	//��ȡ�����趨
	char* old_locale = _strdup(setlocale(LC_CTYPE, NULL));
	//�趨���������ġ�����
	setlocale(LC_CTYPE, "chs");
	//�����ļ������ı���ʽ��
	CStdioFile csdioFile;
	BOOL flag = csdioFile.Open(_T(".\\stock.txt"), CFile::modeReadWrite);
	//д�������������
	if (type.IsEmpty())
	{
		csdioFile.WriteString(_T("��"));
	}
	else
		csdioFile.WriteString(type);
	
	//�ָ������趨
	setlocale(LC_CTYPE, old_locale);
	free(old_locale);

	/*const char* query;
	char temp[10];
	::wsprintfA(temp, "%ls", (LPCTSTR)type);
	query = temp;*/
	
	csdioFile.WriteString(_T(" "));
	csdioFile.WriteString(DoubleToCString(x_floor));
	csdioFile.WriteString(_T(" "));
	csdioFile.WriteString(DoubleToCString(x_ceil));
	csdioFile.WriteString(_T(" "));
	csdioFile.WriteString(DoubleToCString(y_floor));
	csdioFile.WriteString(_T(" "));
	csdioFile.WriteString(DoubleToCString(y_ceil));
	csdioFile.WriteString(_T(" "));
	csdioFile.WriteString(DoubleToCString(theta_floor));
	csdioFile.WriteString(_T(" "));
	csdioFile.WriteString(DoubleToCString(theta_ceil));
	//csdioFile.WriteString(_T("\r\n"));
	/*MidData = SendFreqData[4];
	char MyChar[10];
	_itoa_s(MidData, MyChar, 10);*/

	csdioFile.Close();
	//char str[1024];
	//wsprintfA(str, "%ls", type);
	//
	//ofs.write(str, type.GetLength());
	////ofs <<" " << type << " ";
	//ofstream ofs(_F_STOCK);
	//ofs <<" " << x_floor << " ";
	//ofs << x_ceil << " ";
	//ofs << y_floor << " ";
	//ofs << y_ceil << " ";
	//ofs << theta_floor << " ";
	////ofs << theta_ceil << " ";
	//ofs << theta_ceil << endl;
	
	

	//ofs.close();//�ر��ļ�
}

//�������Ʒ
//name:��Ʒ���ƣ�num����棬price���۸�
//void CInfoFile::Addline(CString name, int num, int price)
//{
//	msg tmp;
//
//	if (ls.size() > 0)
//	{
//		//��Ʒ���ƣ���棬�۸���Ч
//		if (!name.IsEmpty() && num > 0 && price > 0)
//		{
//			tmp.id = ls.size() + 1;	//id�Զ���1
//			CStringA str;
//			str = name;	//CStringתCStirngA
//			tmp.name = str.GetBuffer(); //CStirngAתchar *����Ʒ����
//			tmp.num = num;	//���
//			tmp.price = price;	//�۸�
//
//			ls.push_back(tmp);	//��������ĺ���
//		}
//	}
//}








CString CInfoFile::DoubleToCString(double x)
{
	// TODO: �ڴ˴����ʵ�ִ���.

	char temp[15];
	CString sTemp;
	if ((x - (int)x) == 0)
	{
		_itoa_s(x, temp, 10);
	}
	else
		_gcvt_s(temp, 15, x, 10);//������תΪ�ַ���
	sTemp = CA2CT(temp);
	return sTemp;
}
