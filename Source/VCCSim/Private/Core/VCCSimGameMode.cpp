/*
* Copyright (C) 2025 Visual Computing Research Center, Shenzhen University
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "Core/VCCSimGameMode.h"
#include "Core/VCCHUD.h"
#include "IImageWrapperModule.h"

AVCCSimGameMode::AVCCSimGameMode(const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
	DefaultPawnClass = nullptr;
	HUDClass = AVCCHUD::StaticClass();
}

void AVCCSimGameMode::StartPlay()
{
    Super::StartPlay();
    UserSettings = GEngine->GetGameUserSettings();
}