#include <iostream>
#include <string>
#include <algorithm>
using namespace std;

#include "olcConsoleGameEngine.h"

class GiatThuatAStar : public olcConsoleGameEngine
{
public:
	GiatThuatAStar()
	{
		m_sAppName = L"Giai thuat A*";
	}

private:

	struct sNode
	{
		bool laVatCan = false;			// true là vật cản, false là không
		bool daDuyet = false;			// true là đã duyệt, false là chưa
		float f;						// f(p) = g(p) + h(p)
		float g;						// khoảng cách từ trạng thái đầu đến trạng thái hiện tại
		int x;							// vị trí x, y của nút trong tọa độ 2D
		int y;
		vector<sNode*> nutHangXom;		// vector chứa các nút hàng xóm
		sNode* nutCha;					// nút cha
	};

	sNode* nodes = nullptr;
	int nMapWidth = 10;
	int nMapHeight = 10;

	sNode* nutBatDau = nullptr;
	sNode* nutKetThuc = nullptr;


public:
	virtual bool OnUserCreate()
	{
		// khởi tạo mảng một chiều chứa tọa độ các nút
		nodes = new sNode[nMapWidth * nMapHeight];
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				nodes[y * nMapWidth + x].x = x;
				nodes[y * nMapWidth + x].y = y;
				nodes[y * nMapWidth + x].laVatCan = false;
				nodes[y * nMapWidth + x].nutCha = nullptr;
				nodes[y * nMapWidth + x].daDuyet = false;
			}

		// lưu các hàng xóm vào vector
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				if (y > 0)
					//hàng xóm phía trên
					nodes[y * nMapWidth + x].nutHangXom.push_back(&nodes[(y - 1) * nMapWidth + (x + 0)]);
				if (y < nMapHeight - 1)
					//hàng xóm phía dưới
					nodes[y * nMapWidth + x].nutHangXom.push_back(&nodes[(y + 1) * nMapWidth + (x + 0)]);
				if (x > 0)
					//hàng xóm bên trái
					nodes[y * nMapWidth + x].nutHangXom.push_back(&nodes[(y + 0) * nMapWidth + (x - 1)]);
				if (x < nMapWidth - 1)
					//hàng xóm bên phải
					nodes[y * nMapWidth + x].nutHangXom.push_back(&nodes[(y + 0) * nMapWidth + (x + 1)]);
			}

		//khởi tạo các nút start, end, vật cản mặc định khi mới chạy chương trình
		nutBatDau = &nodes[0 * nMapWidth + 0];
		nutKetThuc = &nodes[4 * nMapWidth + 6];

		int x_vatCan[10] = { 2, 2, 2, 3, 4, 4, 4, 4, 4, 5 };
		int y_vatCan[10] = { 1, 2, 3, 6, 2, 3, 4, 5, 6, 2 };
		for (int i = 0; i < 10; i++)
		{
			nodes[y_vatCan[i] * nMapWidth + x_vatCan[i]].laVatCan = true;
		}
		giai_thuat_AStar();
		return true;
	}

	bool giai_thuat_AStar()
	{
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				nodes[y * nMapWidth + x].daDuyet = false;
				nodes[y * nMapWidth + x].nutCha = nullptr;
				nodes[y * nMapWidth + x].g = INFINITY;
				nodes[y * nMapWidth + x].f = INFINITY;
			}

		auto cost = [](sNode* p, sNode* q) //cost(p, q): là khoảng cách giữa p, q.
		{
			return sqrtf((p->x - q->x) * (p->x - q->x) + (p->y - q->y) * (p->y - q->y));
		};

		auto heuristic = [cost](sNode* q, sNode* dich)
		{
			//h(q): giá trị được lượng giá từ trạng thái hiện tại đến trạng thái đích.
			return cost(q, dich);
		};

		sNode* nutHienTai = nutBatDau;
		nutBatDau->g = 0.0f;
		nutBatDau->f = heuristic(nutBatDau, nutKetThuc);

		list<sNode*> Open; //danh sách chứa các trạng thái đã được sinh ra
		Open.push_back(nutBatDau);

		while (!Open.empty() && nutHienTai != nutKetThuc)
		{
			//sắp xếp lại rồi mới kiểm tra
			Open.sort([](const sNode* lhs, const sNode* rhs) { return lhs->f < rhs->f; });

			//xóa những nút đã duyệt nằm ở đầu danh sách
			while (!Open.empty() && Open.front()->daDuyet)
				Open.pop_front();

			//danh sách hết thì thoát
			if (Open.empty())
				break;

			nutHienTai = Open.front();
			nutHienTai->daDuyet = true;


			//kiểm tra các nút hàng xóm
			for (auto hangXom : nutHienTai->nutHangXom)
			{
				//bỏ các nút chưa được duyệt và không phải vật cản vào Open
				if (!hangXom->daDuyet && hangXom->laVatCan == false)
					Open.push_back(hangXom);

				//tính toán tổng chi phí đường đi từ chỗ hiện tại đến node tiếp theo
				//nếu mà node tiếp theo có chi phí g lớn hơn thì thay thế
				float g = nutHienTai->g + cost(nutHienTai, hangXom);

				//nếu g(p) + cost(p, q) < g(q) 
				if (g < hangXom->g)
				{
					hangXom->nutCha = nutHienTai;
					hangXom->g = g;
					hangXom->f = hangXom->g + heuristic(hangXom, nutKetThuc);
				}
			}
		}
		return true;
	}

	void in_duong_di()
	{
		if (nutKetThuc != nullptr)
		{
			cout << "Nut bat dau: (" << nutBatDau->x << ", " << nutBatDau->y << ")" << "\n";
			cout << "Nut ket thuc: (" << nutKetThuc->x << ", " << nutKetThuc->y << ")" << "\n";
			cout << "Duong di:\n";
			sNode* p = nutKetThuc;
			while (p != nullptr)
			{
				cout << "(" << p->x << ", " << p->y << ")" << "\n";
				p = p->nutCha;
			}
		}
	}

	virtual bool OnUserUpdate(float fElapsedTime)
	{
		int nutSize = 9;
		int nutBorder = 1;

		//tính tọa độ của nút được click
		int x_nut = m_mousePosX / nutSize;
		int y_nut = m_mousePosY / nutSize;

		if (m_mouse[0].bReleased)
		{
			if (x_nut >= 0 && x_nut < nMapWidth)
				if (y_nut >= 0 && y_nut < nMapHeight)
				{
					if (m_keys[VK_SHIFT].bHeld)
						nutBatDau = &nodes[y_nut * nMapWidth + x_nut];
					else if (m_keys[VK_CONTROL].bHeld)
						nutKetThuc = &nodes[y_nut * nMapWidth + x_nut];
					else
						nodes[y_nut * nMapWidth + x_nut].laVatCan = !nodes[y_nut * nMapWidth + x_nut].laVatCan;

					giai_thuat_AStar();
				}
		}

		//tô màu ma trận
		Fill(0, 0, ScreenWidth(), ScreenHeight(), L' ');

		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{

				Fill(x * nutSize + nutBorder, y * nutSize + nutBorder,
					(x + 1) * nutSize - nutBorder, (y + 1) * nutSize - nutBorder,
					PIXEL_SOLID, nodes[y * nMapWidth + x].laVatCan ? FG_DARK_GREY : FG_WHITE);

				if (nodes[y * nMapWidth + x].daDuyet)
					Fill(x * nutSize + nutBorder, y * nutSize + nutBorder,
						(x + 1) * nutSize - nutBorder, (y + 1) * nutSize - nutBorder, PIXEL_SOLID, FG_BLUE);

			}

		// vẽ đường đi ngắn nhất từ nút kết thúc về nút bắt đầu nhờ thuộc tính nutCha sau khi chạy thuật toán
		if (nutKetThuc != nullptr)
		{
			sNode* p = nutKetThuc;
			while (p != nullptr)
			{
				Fill(p->x * nutSize + nutBorder, p->y * nutSize + nutBorder,
					(p->x + 1) * nutSize - nutBorder, (p->y + 1) * nutSize - nutBorder, PIXEL_SOLID, FG_YELLOW);
				//cout << "(" << p->x << ", " << p->y << ")" << "\t";
				p = p->nutCha;
			}
		}

		//tô màu nút bắt đầu
		Fill(nutBatDau->x * nutSize + nutBorder, nutBatDau->y * nutSize + nutBorder,
			(nutBatDau->x + 1) * nutSize - nutBorder, (nutBatDau->y + 1) * nutSize - nutBorder, PIXEL_SOLID, FG_GREEN);

		//tô màu nút kết thúc
		Fill(nutKetThuc->x * nutSize + nutBorder, nutKetThuc->y * nutSize + nutBorder,
			(nutKetThuc->x + 1) * nutSize - nutBorder, (nutKetThuc->y + 1) * nutSize - nutBorder, PIXEL_SOLID, FG_RED);
		return true;
	}

};

int main()
{
	GiatThuatAStar game;
	game.ConstructConsole(100, 100, 5, 5);
	game.Start();
	//game.OnUserCreate();
	//game.in_duong_di();
	return 0;
}