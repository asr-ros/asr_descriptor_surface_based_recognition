/**

Copyright (C) 2016, Allgeyer Tobias, Hutmacher Robin, Meißner Pascal

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#ifndef WXIMAGEPANEL_H_
#define WXIMAGEPANEL_H_

#include <wx/wx.h>


class wxImagePanel : public wxPanel {

private:
    wxBitmap* image;

public:
    wxImagePanel(wxWindow* parent, int height, int width);

    void paintEvent(wxPaintEvent & evt);
    void paintNow();

    void render(wxDC& dc);

    void setImage(wxBitmap* image);


    DECLARE_EVENT_TABLE()
};



#endif /* WXIMAGEPANEL_H_ */
